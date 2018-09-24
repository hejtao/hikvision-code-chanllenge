# -*- coding:utf-8 -*-
import sys
import socket
import json
import heapq
import random
import time

#从服务器接收一段字符串, 转化成字典的形式
def RecvJuderData(hSocket):
    nRet = -1
    Message = hSocket.recv(1024 * 1024 * 4)
    print(Message)
    len_json = int(Message[:8])
    str_json = Message[8:].decode()
    while len(str_json) != len_json:
        Message = hSocket.recv(1024*1024*4)
        str_json = str_json + Message.decode()

    nRet = 0
    Dict = json.loads(str_json)
    #print(Dict["map"]["UAV_we"])


    return nRet, Dict

# 接收一个字典,将其转换成json文件,并计算大小,发送至服务器
def SendJuderData(hSocket, dict_send):
    str_json = json.dumps(dict_send)
    len_json = str(len(str_json)).zfill(8)
    str_all = len_json + str_json
    print(str_all)
    ret = hSocket.sendall(str_all.encode())
    if ret == None:
        ret = 0
    print('sendall', ret)
    return ret
###########################################################
'''
己方飞机的类，包括用到的参数oneuav：一架飞机的字典，Map：整个地图，
'''
class planetype(object):
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.pos=[]
        self.no = []
        self.type = []
        self.target_goods_no = []
        self.target_goods_pos = []
        self.target_goods_value = []
        self.target_goods_desti = []
        self.enemy_no = []
        self.virtual_enemy_no=[]
        self.virtual_enemy_pos=[]
        self.virtual_goods_no =[]
        self.virtual_goods_pos=[]
        self.enemy_desti=[]
        self.path = []
        self.pre_path=[]
        self.load_weight = []
        self.plane_value = []
        self.all_value = []
        self.status=[]
        self.capacity=[]
        self.charge=[]
        self.Is_full=[]			#判断电量是否充满
        self.do_charge ="do"    		#判断飞机是否需要充电，战斗机不需充电
        self.attack_home =[]

    #用于得到飞机最基本的信息，一旦得到不会再变
    def get_type(self, oneuav, Map):			#这里的oneuav是来自"UAV_we"（列表）中的一架飞机，用字典表示
        self.type = oneuav["type"]
        self.no = oneuav["no"]
        for i in range(0, len(Map["UAV_price"])):
            if  self.type == Map["UAV_price"][i]["type"]:
                self.load_weight = Map["UAV_price"][i]["load_weight"]
                self.plane_value = Map["UAV_price"][i]["value"]
                self.capacity=Map["UAV_price"][i]["capacity"]
                self.charge = Map["UAV_price"][i]["charge"]

    ###用于获得飞机实时变化的信息
    def update(self, oneuav, cheapest_type,time,change_time):
        self.type = oneuav["type"]			#虽然重复登记，但是有些情况，上面函数用不着，可直接调用本函数
        self.no = oneuav["no"]
        self.x = oneuav["x"]
        self.y = oneuav["y"]
        self.z = oneuav["z"]
        self.pos = (oneuav["x"],oneuav["y"],oneuav["z"])
        self.goods_no = oneuav["goods_no"]			#表示无人机载货情况，-1表示空载
        self.status = oneuav["status"]			#0.1,2分别表示正常，坠毁，雾区
        self.change_target="NO"
        self.remain_electricity=oneuav["remain_electricity"]
        self.fighting_place=[]			#包括 “put_goods” “get_goods”  “enemy_home” “put_goods” "guarding"  分别表示攻击放货点，攻击取货点，攻击敌人家和充电飞机（俯冲下来，俯冲轰炸机），护卫机

        if time >=change_time:
            if  self.type == cheapest_type and self.goods_no == -1:			#暂时定为这样就切换成战斗机
                 self.role="fighting"			#trans表示运输机，fighting表示战斗机
                 self.do_charge = "donot"
            else:
                self.role = "trans"
        else:
            self.role="trans"


    def get_goods_info(self, goodslist):
        for i in goodslist:
            if  self.target_goods_no == i["no"]:
                self.target_goods_pos = (i["start_x"], i["start_y"],0)
                self.target_goods_desti = (i["end_x"], i["end_y"], 0)
                self.target_goods_value = i["value"]
                self.target_goods_weight = i["weight"]


    def allvalue(self, goodlist):
         if self.goods_no == -1:
             self.all_value =self.plane_value
             #print("有人吗")
         else:
             for  i in goodlist:
                 if self.goods_no == i["no"]:
                    self.all_value = self.plane_value + i["value"]
                    #print("有人，货物价值为", i["value"])

    def decision(self):
         if self.status ==1:			#坠毁
             self.act=-1
         else:
             if  self.role =="trans":
                 if self.goods_no == -1:			#空载，决定是否要充电
                         self.act=1			#找货物
                 else:
                     self.act=2			#路径规划，送货到地点
             else:
                 self.act=3			#战斗机，不需要充电





#################################################################
'''
定义地图类，包括范围，障碍物，雾区等
'''
class MAP(object):
    def __init__(self, Map):
        self.x_start = 0
        self.x_end = Map["map"]["x"]-1
        self.y_start = 0
        self.y_end = Map["map"]["y"] - 1
        self.z_start = 0
        self.z_end = Map["map"]["z"] - 1
        self.low = Map["h_low"]
        self.high = Map["h_high"] -1
        self.parking_pos = (Map["parking"]["x"], Map["parking"]["y"],0)
        self.building = Map["building"]			#是列表，列表的每个元素为一个字典，表示一个building
        self.fog = Map["fog"]

    def Inbound(self, point):			#判断是否在地图界内
        (x, y, z)=(point[0], point[1], point[2])
        return self.x_start <=x<= self.x_end  and self.y_start <=y<= self.y_end and self.z_start <=z<= self.high

    def Infog(self, point):			#判断是否在building内
        (x, y, z)= (point[0], point[1], point[2])
        in_fog= 0
        for i in self.fog:
            if i["x"]<=x<=i["x"]+i["l"]-1 and i["y"]<=y<=i["y"]+i["w"]-1 and i["b"]-1<=z<=i["t"]-1:
                in_fog = 1
                return True
        if in_fog ==0:
            return False

    def Outbuilding(self, point):			#判断是否在building内
        (x, y, z) = (point[0], point[1], point[2])
        passable = 1
        for i in self.building:
            if i["x"]<=x<=i["x"]+i["l"]-1 and i["y"]<=y<=i["y"]+i["w"]-1 and 0<=z<=i["h"]-1:
                passable = 0
                return False
        if passable==1:
            return True

    def neighbors(self, point, target):			#给出周围可行点
        (x, y, z) = (point[0], point[1], point[2])
        (t_x, t_y, t_z)=(target[0], target[1], target[2])
        if z<self.low:
            results=[(x, y, z+1), (x, y, z-1)]
        else:
            if z>self.low:
                 results=[(x, y, z+1), (x, y, z-1), (x+1, y, z),  (x+1, y+1, z), (x+1, y-1, z),
                     (x, y-1, z), (x, y+1, z), (x-1, y+1, z), (x-1, y-1, z), (x-1, y, z),(x,y,z)]
            else:
                if   x==t_x and y==t_y:
                    results = [(x, y, z + 1), (x, y, z - 1), (x + 1, y, z), (x + 1, y + 1, z), (x + 1, y - 1, z),
                           (x, y - 1, z), (x, y + 1, z), (x - 1, y + 1, z), (x - 1, y - 1, z), (x - 1, y, z),(x,y,z)]
                else:
                    results = [(x, y, z + 1), (x + 1, y, z), (x + 1, y + 1, z), (x + 1, y - 1, z),
                           (x, y - 1, z), (x, y + 1, z), (x - 1, y + 1, z), (x - 1, y - 1, z), (x - 1, y, z),(x,y,z)]

        results = filter(self.Inbound, results)
        results = filter(self.Outbuilding, results)
        return results


#############################################################
'''
定义商品类，包括起点终点，货物重点价值剩余时间
'''
class goodsInfo(object):
    def __init__(self, Goods):			#这里的Good是一个字典，是给本机传信息的goods列表的一个（字典）
        self.no = Goods["no"]
        self.start_x = Goods["start_x"]
        self.start_y = Goods["start_y"]
        self.start_z = 0
        self.start_pos = (Goods["start_x"], Goods["start_y"], 0)
        self.end_x = Goods["end_x"]
        self.end_y = Goods["end_y"]
        self.end_pos = (Goods["end_x"], Goods["end_y"], 0)
        self.weight = Goods["weight"]
        self.value = Goods["value"]
        self.left_time = Goods["left_time"]
        self.status = Goods["status"]

##############################################################
'''
定义UAV的价格清单
'''
class uavlist(object):
    def __init__(self, uavinfo):
        self.type=uavinfo["type"]
        self.load_weight=uavinfo["load_weight"]
        self.value=uavinfo["value"]
        self.capacity = uavinfo["capacity"]
        self.charge = uavinfo["charge"]
        self.ratio = self.load_weight/self.value    	#用于购买飞机，现在看来要修改啊

##############################################################
'''
从这里可以看出plane_set以及goods_val都是字典，键分别为飞机编号和货物编号。
'''
def get_goods_avl(Match_Status):
    goods_avl={}
    for i in Match_Status["goods"]:
        g=goodsInfo(i)
        if g.status == 0:
            goods_avl[i["no"]]=g
    return goods_avl

def get_planes_set(Match_Status, Map, plane_set, cheapest_type,time,enemy_parking, attack_home_path,change_time):  		 #本函数没有给出飞机是否需要充电的判断，对于运输机，是否充电应当在选货物的时候判断
    count = 100
    if enemy_parking !=[]:
        count =0
        for i in plane_set:
            if plane_set[i].attack_home == "attack":
                count += 1
    #print("zheyoushishenme ne", attack_home_path)
    for i in Match_Status["UAV_we"]: #这里的i为列表中的元素，即一个字典
        if i["status"] != 1:
            if i["no"] not in plane_set:
                uav = planetype()    	#定义我方飞机类
                uav.get_type(i,Map)
                uav.update(i, cheapest_type,time,change_time)
                uav.allvalue(Match_Status["goods"])
                uav.decision()
                if count < 2:
                    if uav.role == "fighting":
                        uav.attack_home ="attack"
                        uav.pre_path = attack_home_path.copy()
                plane_set[i["no"]] = uav  		# 这里表示plane_set的键是飞机编号，值是飞机类
            else:
                plane_set[i["no"]].update(i, cheapest_type,time,change_time)
                plane_set[i["no"]].allvalue(Match_Status["goods"])
                plane_set[i["no"]].decision()    		#如果没有新的飞机，那么判断当前飞机状态
        elif i["no"]  in plane_set:     	#这个表示如果飞机status==1，飞机坠毁还在plane_set中，那么删除它
            del plane_set[i["no"]]
    #for i in plane_set:
         #print("wojiukankanakanakankanakanakana", i, plane_set[i].attack_home, plane_set[i].pos, plane_set[i].pre_path)
    return plane_set

def get_enemy_plane(Match_Status, Map, cheapest_type,time, change_time):
    enemy_plane_set ={}
    for i in Match_Status["UAV_enemy"]:
        if i["status"] ==0:            #只将看得到的飞机作为敌机，雾区的飞机不做考虑
            uav = planetype()
            uav.get_type(i, Map)
            uav.update(i, cheapest_type, time,change_time)
            for j in Match_Status["goods"]:
                if uav.goods_no == j["no"]:
                    uav.target_goods_desti = (j["end_x"], j["end_y"], 0)
            enemy_plane_set[i["no"]] = uav
        else:
            uav = planetype()
            uav.x=-10
            uav.y=-10
            uav.z=-10
            uav.pos=(-10,-10,-10)
            uav.type =i["type"]
            uav.goods_no = i["goods_no"]
            for j in Match_Status["goods"]:
                if uav.goods_no == j["no"]:
                    uav.target_goods_desti = (j["end_x"], j["end_y"], 0)
            enemy_plane_set[i["no"]] = uav

    return enemy_plane_set

'''储存敌方所有信息，包括坠毁，正常，雾区，充电'''
def get_all_enemy_plane(Match_Status, Map, all_enemy_plane, cheapest_type, time,change_time):
    for i in Match_Status["UAV_enemy"]:
        if i["no"] not in all_enemy_plane:   		#敌人新的飞机，做初始化，更新敌机信息
            uav=planetype()
            uav.get_type(i, Map)
            uav.update(i, cheapest_type, time,change_time)
            uav.status = i["status"]
            all_enemy_plane[i["no"]]=uav
        else:          #敌机已经考虑过了，重新更新敌机信息
            all_enemy_plane[i["no"]].update(i, cheapest_type, time,change_time)
            all_enemy_plane[i["no"]].status = i["status"]

    return all_enemy_plane

'''敌方活着的飞机数目'''
def get_enemy_num(Match_Status):
    enemy_num = 0
    for i in Match_Status["UAV_enemy"]:
        if i["status"] !=1:
            enemy_num+=1
    return enemy_num



##############################################################

'''此函数规定了两个坐标点的距离'''
def Distance(a,b):
    dist = max(abs(a[0]-b[0]), abs(a[1]-b[1])) + abs(a[2]-b[2])
    return dist

'''此函数用于路径规划'''
def AstarSearch(map,  start, goal, plane_set, enemy_plane_set, role): #role表示无人机的角色
    openlist = []
    heapq.heappush(openlist,(0,start))
    camefrom = {}
    costsofar={}
    camefrom[start] = None
    costsofar[start] = 0

    while len(openlist) !=0 :
        current = heapq.heappop(openlist)[1]
        if current == goal:
            break

        results = map.neighbors(current, goal)
        uav_we_nei=ouruavneighbor(plane_set, start)

        buffer=[]
        for i in results:
            if  i not in uav_we_nei:
                buffer.append(i)

        results = buffer

        if  role == "trans":
            buffer=[]
            uav_enemy_nei = enemyuavneighbor(enemy_plane_set, start, map)
            #print("敌人在哪",uav_enemy_nei )
            for i in results:
                if i not in uav_enemy_nei:
                    buffer.append(i)

            results = buffer

        #print("哪里可以走", results)


        for next in results:
            dis = Distance(next, goal)
            newcost = costsofar[current] + 1
            if next not in costsofar or newcost < costsofar[next]:
                costsofar[next] = newcost
                priority = newcost + dis
                heapq.heappush(openlist,(priority, next))
                camefrom[next] = current

        #print("查看当前无人机邻居", uav_we_nei, "可以移动方向", results)

    return camefrom

def restruct_path(camefrom, start, goal):                 #把closelist传给camefrom
    #print( 123355622, camefrom, start, goal)
    current = goal
    path=[current]
    while current != start:
        current = camefrom[current]
        path.append(current)

    path.reverse()
    if path[0] == start:
        path.pop(0)

    return path



'''本函数是为了找到本机位置附近所有无人机的位置以及可能发生碰撞的点'''
def ouruavneighbor(plane_set, point): 			#point是元组，plane_set[i].pos是列表，产生的uav_we_nei是列表的列表
    (x,y,z)=(point[0], point[1], point[2]) 			#point是飞机当前位置，注意plane_set[i].pos在不同的迭代周期内是可以会变得，每次迭代完都会更新飞机位置。
    uav_we_nei=[]
    for i in plane_set:
        if plane_set[i].pos != (x,y,z) and Distance(point, plane_set[i].pos)<=2: 			#与本机的距离小于3的所有飞机作为障碍
            uav_we_nei.append(plane_set[i].pos)
            bux=[plane_set[i].pos[0],  plane_set[i].x]
            buy=[plane_set[i].pos[1],  plane_set[i].y]
            if x in  bux and y in buy and plane_set[i].pos[2] == z:   			 #这个是为了防止对角碰撞
                bux.remove(x)
                buy.remove(y)
                point=(bux[0],buy[0], z)
                uav_we_nei.append(point)

    return uav_we_nei


'''将本机附近的敌机的位置取出来，用于运输机的避障'''
def enemyuavneighbor(enemy_plane_set, point, map): 			#point是元组，plane_set[i].pos是列表，产生的uav_we_nei是列表的列表
    uav_enemy_nei=[]
    for i in enemy_plane_set:
        if enemy_plane_set[i].pos != []:
            if Distance(point, enemy_plane_set[i].pos)<=2:
                uav_enemy_nei +=map.neighbors(enemy_plane_set[i].pos, enemy_plane_set[i].pos)

    uav_enemy_nei=list(set(uav_enemy_nei))
    return uav_enemy_nei

################################################
################################################
'''选择货物的函数'''
def selectgood(map, plane_set, enemy_plane_set, plane, goods_avl):  		 #goods_avl可用商品的字典,键为货物编号，值为货物信息。,此函数只要获得货物编号即可。
    previous_goods_no = plane.target_goods_no	
    plane.target_goods_no = []
    choice_para = 100000
    w0=0.05
    w1 =0.75
    w2= 0.05
    w3=0.1
    w4=0.05
    sum_taking =0
    sum_all_dis=0
    sum_value=0
    sum_weight=0
    sum_consume=0
    plane_avlgoods={}

    for i in goods_avl:
        dist_trans = Distance(goods_avl[i].start_pos, goods_avl[i].end_pos) + 2 * map.high
        dis_taking = Distance(plane.pos, goods_avl[i].start_pos)
        esti_consume = dist_trans * goods_avl[i].weight
        if plane.load_weight>=goods_avl[i].weight and dis_taking<=goods_avl[i].left_time and esti_consume< plane.remain_electricity:
            dist_all = dist_trans + dis_taking
            sum_taking += dis_taking
            sum_all_dis += dist_all
            sum_value += goods_avl[i].value
            sum_weight += goods_avl[i].weight
            sum_consume +=esti_consume
            plane_avlgoods[i] =[goods_avl[i].no, goods_avl[i].start_pos, goods_avl[i].end_pos, goods_avl[i].weight, goods_avl[i].value, esti_consume, dis_taking, dist_all]

    num=len(plane_avlgoods)
    #print("shenme qingkuang ",plane_avlgoods, num, plane.goods_no, plane.path)
    if num !=0:  #进行选货
        horizon_distribute = []
        for i in plane_set:
            if plane_set[i].pos != plane.pos:
                b = (plane_set[i].pos[0], plane_set[i].pos[1])
                horizon_distribute.append(b)
        for j in enemy_plane_set:
            if enemy_plane_set[j].pos != []:
                c = (enemy_plane_set[j].pos[0], enemy_plane_set[j].pos[1])
                horizon_distribute.append(c)

        ave_taking = sum_taking/num
        ave_all_dis = sum_all_dis/num
        ave_value = sum_value/num
        ave_weight = sum_weight/num
        ave_consume = sum_consume/num
        #print("youmeiyouyunxing 0", plane.target_goods_no)
        for i in plane_avlgoods:
            choice0= w0*min(ave_weight/plane_avlgoods[i][3], 1)
            choice1 = w1*min(ave_value/plane_avlgoods[i][4], 1)
            choice2= w2* min(plane_avlgoods[i][5]/ave_consume,1)
            choice3 = w3*min(plane_avlgoods[i][6]/ave_taking, 1.5)
            choice4 = w4*min(plane_avlgoods[i][7]/ave_all_dis, 1)
            choice = choice0+choice1+choice2+choice3+choice4
            c=(plane_avlgoods[i][1][0], plane_avlgoods[i][1][1])
            if c==(plane.x,plane.y) and plane.z<map.low:
                plane.target_goods_no = plane_avlgoods[i][0]
                plane.target_goods_pos = plane_avlgoods[i][1]
                plane.target_goods_desti = plane_avlgoods[i][2]
                plane.target_goods_weight = plane_avlgoods[i][3]
                #print("youmeiyouyunxing 1",plane.target_goods_no)
                break
            else:
                est_we = c not in horizon_distribute
                #print("youmeiyouyunxing 444", est_we)
                if est_we and choice<choice_para:
                    choice_para = choice
                    plane.target_goods_no = plane_avlgoods[i][0]
                    plane.target_goods_pos = plane_avlgoods[i][1]
                    plane.target_goods_desti = plane_avlgoods[i][2]
                    plane.target_goods_weight = plane_avlgoods[i][3]
                    #print("youmeiyouyunxing 2",plane.target_goods_no)

    #print("我方无人机分布", horion_distribute, "选了啥", plane.target_goods_no)
    if plane.target_goods_no != []:
        del goods_avl[plane.target_goods_no]

    if plane.target_goods_no == previous_goods_no:
        plane.change_target= "NO"
    else:
        plane.change_target ="YES"   #表示修改货物目标，需要重新规划
    return plane

###################################################################

'''对无人机进行优先级排序，有货的运输机>战斗机>空载无需充电运输机>停机坪上空载无需充电运输机>空载需充电运输机'''
def sortplane(plane_set, map,sort_plane):
    load_plane=[]
    fighting_plane=[]
    donot_charge_plane=[]
    parking_donot_charge_plane = []
    do_charge_plane=[]
    copy_plane_set = plane_set.copy()
    for i in sort_plane:
        if i in plane_set:  #表示i的飞机不能已经撞毁
            if plane_set[i].role=="fighting":
                fighting_plane.append(i)
            else:
                if plane_set[i].goods_no != -1:
                    load_plane.append(i)
                else:
                    if plane_set[i].do_charge =="donot":
                        if plane_set[i].x == map.parking_pos[0] and plane_set[i].y == map.parking_pos[1] and plane_set[i].z ==0:
                            parking_donot_charge_plane.append(i)
                        else:
                            donot_charge_plane.append(i)
                    else:
                        do_charge_plane.append(i)
            del copy_plane_set[i]

    for i in copy_plane_set:
        if plane_set[i].role=="fighting":
            fighting_plane.append(i)
        else:
            if plane_set[i].goods_no != -1:
                load_plane.append(i)
            else:
                if plane_set[i].do_charge =="donot":
                    if plane_set[i].x == map.parking_pos[0] and plane_set[i].y == map.parking_pos[1] and plane_set[i].z ==0:
                        parking_donot_charge_plane.append(i)
                    else:
                        donot_charge_plane.append(i)
                else:
                    do_charge_plane.append(i)

    sort_plane= load_plane + fighting_plane + donot_charge_plane + parking_donot_charge_plane + do_charge_plane
    #print("我方飞机排序", sort_plane)
    return sort_plane

####################################################################
'''对无人机列表进行排序，挑选战斗机'''
def sortuavratio(uavlist):
    sort_ratio_uav=[]
    copy_uavlist = uavlist.copy()
    for i in uavlist:
        buf3 = 0
        buf4 = "F1"
        for j in copy_uavlist:
            if copy_uavlist[j].ratio >= buf3:
                buf3 = copy_uavlist[j].ratio
                buf4 = copy_uavlist[j].type

        sort_ratio_uav.append(buf4)
        del copy_uavlist[buf4]
    return sort_ratio_uav

def lessvalue(uavlist):
    b=10000
    for i in uavlist:
        if uavlist[i].value < b:
            b =uavlist[i].value
            cheapest_type = uavlist[i].type
            cheapest_value = uavlist[i].value

    #print("最便宜的是", cheapest_type, cheapest_value)
    return cheapest_type, cheapest_value
####################################################################

'''定义购买无人机函数'''
def purchase(sort_ratio_uav, cheapest_type, cheapest_value, uavlist, pstMapInfo, pstMatchStatus,plane_set, enemy_plane_set, num_enemy):
    goods_avl=get_goods_avl(pstMatchStatus)
    amount_goods = 0
    all_weight = 0
    for i in goods_avl:
        if goods_avl[i].value > 150:
            all_weight += goods_avl[i].weight
            amount_goods+=1


    mean_weight=100

    if amount_goods>0:
        mean_weight=all_weight/amount_goods

    current_money = pstMatchStatus["we_value"]
    buy = []
     ##################################
    count=0    			#判断我方停机坪上有几架飞机，有，干他

    for j in enemy_plane_set:
        if enemy_plane_set[j].pos !=[]:
            if (enemy_plane_set[j].pos[0], enemy_plane_set[j].pos[1]) == (pstMapInfo["parking"]["x"], pstMapInfo["parking"]["x"]):
                count += 1

    if count !=0:
        buy = []
        a = {"purchase":cheapest_type}
        for k in range(0,count):
            if  current_money>cheapest_value:
                  buy.append(a)
                  current_money -= cheapest_value

            return buy
     ##################################
    else:    			#傻子不来进攻，我采取策略买战斗机和运输机
        num_we=len(plane_set)
        print("我有几架",num_we,"敌人几架",num_enemy)
        if  num_we<8*num_enemy:
            amount_trans=0
            all_total_we_uav=len(plane_set)
            for i in plane_set:
                if plane_set[i].role == "trans":
                    amount_trans+=1

            amount_fighting = all_total_we_uav- amount_trans
            if  amount_fighting <=amount_trans:
                if  current_money>=cheapest_value:
                    buy=[ {"purchase":cheapest_type}]
            else:
                for i in sort_ratio_uav:
                    if uavlist[i].load_weight > mean_weight:
                        if uavlist[i].value <= current_money:
                            buy = [ {"purchase":i}]
        else:
            buy=[]

        return buy



###################################################################
'''计算飞机的几个数目'''
def calculate_plane(plane, map, plane_set, enemy_plane_set):
    count_enemy_come = 0  			# 统计敌人可能会进攻的飞机数
    count_enemy_here = 0  			# 统计敌人进攻进来的飞机数
    count_we_up = 0  			# 统计我方飞机向上准备飞出map.low的飞机数
    count_we_parking_trans = 0  # 统计我方停机场待飞（不需要充电）的运输机数量
    count_we_parking_fighting = 0
    charge_plane_no = []
    h=map.low
    for i in plane_set:
        if (plane_set[i].x, plane_set[i].y) == (map.parking_pos[0], map.parking_pos[1]) and plane_set[i].do_charge == "do" and plane_set[i].z>0:
            if h > plane_set[i].z:
                h = plane_set[i].z

    for i in enemy_plane_set:
        if enemy_plane_set[i].pos !=[]:
            if enemy_plane_set[i].x == map.parking_pos[0] and enemy_plane_set[i].y == map.parking_pos[1] and enemy_plane_set[i].z < h:
                count_enemy_here += 1
            else:
                if Distance(enemy_plane_set[i].pos, [map.parking_pos[0], map.parking_pos[1], map.low]) < map.low and  enemy_plane_set[i].role == "fighting":  			# 这样已经把那些打进来的飞机排除出去了
                    count_enemy_come += 1

    for i in plane_set:
        if plane_set[i].no != plane.no and (plane_set[i].x, plane_set[i].y) == (map.parking_pos[0], map.parking_pos[1]):
            if plane_set[i].z > 0 and plane_set[i].do_charge == "do":  # 不在停机坪上充电的飞机不算
                charge_plane_no.append(i)    # 非空，说明有人要充电，这是要考虑更烦的
            elif 0 < plane_set[i].z < map.low:
                count_we_up += 1
            elif plane_set[i].z == 0:
                if plane_set[i].role == "trans" and plane_set[i].do_charge == "donot":
                    count_we_parking_trans += 1
                else:
                    count_we_parking_fighting += 1

    return count_enemy_come,  count_enemy_here, count_we_up, count_we_parking_trans, count_we_parking_fighting, charge_plane_no


'''
出发函数，进行确认谁先飞出，一旦飞出，要仔细确定是否要飞回。 出发要谨慎！！
出发前，确认敌机会不会来进攻，不会，则运输机上，战斗机靠后；
否则运输机在战斗机之后飞，战斗机的数量由敌机可能进攻的数量决定，如果不够，则需购买
'''

def set_off(plane, map, plane_set, enemy_plane_set):      	#从家里出发的函数
    count_enemy_come, count_enemy_here, count_we_up, count_we_parking_trans, count_we_parking_fighting, charge_plane_no = calculate_plane(plane, map, plane_set, enemy_plane_set)

    print("已经来的敌机",count_enemy_here, "将要来的敌机",count_enemy_come, "我方上升飞机",count_we_up, "停机坪运输机",count_we_parking_trans, "停机坪运输机",count_we_parking_fighting,"chongdianfeiji", charge_plane_no)
    if charge_plane_no==[]:    		#上面没有要充电的
        # 战斗机的飞法, 如果没有待飞的运输机，那就直接上吧；如果有待飞的运输机，判断上面有没有敌人，如果有，且上方的我方飞机数较少，则飞；否则就运输机飞，即本机不飞
        if plane.role == "fighting":
            if plane.z>0:
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                if (plane.x,plane.y,plane.z+1) in uav_we_nei:
                    plane.path = [(plane.x,plane.y,plane.z)]
                else:
                    plane.path = [(plane.x,plane.y,plane.z+1) ]
            else:
                if count_we_parking_trans==0:
                    uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                    print("我的邻居",uav_we_nei)
                    if (plane.x,plane.y,plane.z+1) in uav_we_nei:   			#shangyibu
                        plane.path = [(plane.x,plane.y,plane.z)]
                    else:
                        plane.path = [(plane.x, plane.y, plane.z+1)]
                else:
                    if count_enemy_here > count_we_up:   			#这里除以2的目的是怕估计敌机进攻数量过于保守
                        plane.path = [(plane.x, plane.y, plane.z + 1)]
                    else:
                        plane.path = [(plane.x, plane.y, plane.z)]
        #运输机的飞法。 停机坪上的运输机只在敌人战斗机比我方战斗机少的时候以及我不需要充电时飞；不再停机坪上的，只要避自己的障就好了
        if plane.role == "trans":
            if plane.z==0:
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                print("我的邻居", uav_we_nei)
                if plane.do_charge =="donot" and count_enemy_come/2+count_enemy_here <= count_we_up and (plane.x,plane.y,plane.z+1) not in uav_we_nei:
                    plane.path = [(plane.x,plane.y,plane.z+1)]
                else:
                    plane.path = [(plane.x, plane.y, plane.z)]
            else:
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                if (plane.x,plane.y,plane.z+1) not in uav_we_nei:
                    plane.path = [(plane.x, plane.y, plane.z + 1)]
                else:
                    plane.path =[(plane.x, plane.y, plane.z)]
    # 有人要回来, 战斗机只在count_enemy_here比较大的时候，并且前面没有人挡路的时候飞
    else:
        Fly = "Go"
        for i in charge_plane_no:
            if plane_set[i].z<=map.low:
                Fly="Stop"
                break
            else:
                Fly="Go"
        #对于战斗机，如果是停机坪上的，需要判断我方飞机和敌机个数，如果是停机坪上方，只要不碰撞就往上
        if Fly == "Stop":
            plane.path = [(plane.x, plane.y, plane.z)]
        else:
            if plane.role == "fighting":
                if plane.z==0:
                    uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                    if count_enemy_here > count_we_up and (plane.x,plane.y,plane.z+1) not in uav_we_nei:
                        plane.path = [(plane.x, plane.y, plane.z + 1)]
                    else:
                        plane.path = [(plane.x, plane.y, plane.z)]
                else:
                    uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                    if (plane.x,plane.y,plane.z+1) not in uav_we_nei:
                        plane.path = [(plane.x, plane.y, plane.z + 1)]
                    else:
                        plane.path = [(plane.x, plane.y, plane.z)]
            else:
                if plane.z==0 or (plane.x,plane.y,plane.z+1) in ouruavneighbor(plane_set, plane.pos):
                    plane.path = [(plane.x, plane.y, plane.z)]
                else:
                    plane.path = [(plane.x, plane.y, plane.z + 1)]

    plane.pos = plane.path[0]
    if plane.pos[2] == 0:
        if plane.role == "fighting":
            if plane.remain_electricity +  plane.charge >=plane.capacity:
                plane.remain_electricity=plane.capacity
                plane.do_charge="donot"
            else:
                plane.remain_electricity +=  plane.charge
                plane.do_charge = "donot"
        else:
            if plane.remain_electricity + plane.charge >= plane.capacity:
                plane.remain_electricity = plane.capacity
                plane.do_charge = "donot"
            else:
                plane.remain_electricity += plane.charge
                plane.do_charge = "do"
    return plane



'''定义充电飞机的历程'''
def go_charge(plane, map, plane_set, enemy_plane_set,role, uav_enemy_nei):
    e_here = 0  		# 统计敌人进攻进来的飞机数
    w_up = 0  			# 统计我方飞机向上准备飞出map.low的飞机数
    w_parking_fighting = 0
    for i in enemy_plane_set:
        if enemy_plane_set[i].pos !=[]:
            if enemy_plane_set[i].x == map.parking_pos[0] and enemy_plane_set[i].y == map.parking_pos[1] and enemy_plane_set[i].z < plane.z:
                e_here += 1

    for i in plane_set:
        if plane_set[i].no != plane.no and (plane_set[i].x, plane_set[i].y) == (map.parking_pos[0], map.parking_pos[1]):
            if 0<plane_set[i].z < plane.z and plane_set[i].do_charge == "donot":
                w_up +=1
            else:
                if plane_set[i].z == 0 and plane_set[i].role == "fighting":
                    w_parking_fighting +=1

    if plane.pos[0]==map.parking_pos[0] and plane.pos[1]==map.parking_pos[1]:   		#在停机坪上空
        if plane.pos[2]>1:
            Exist = "No"
            print("有没有飞机1", Exist)
            if e_here > w_up + w_parking_fighting:
                Exist = "Yes"

            if w_up > e_here:
                Exist = "Yes"

            print("有没有飞机3", Exist)
            print("飞机是去充电的吗", plane.do_charge)


            # for i in plane_set:
            #     if i != plane.no and (map.parking_pos[0], map.parking_pos[1]) == (plane_set[i].x, plane_set[i].y) and 0<plane_set[i].z< plane.z and plane_set[i].do_charge == "donot":
            #         Exist = "Yes"
            #         break

            if Exist == "Yes":   		#可能需要适当修改
                plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2]) ]
            else:
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                if (plane.pos[0], plane.pos[1], plane.pos[2]-1) in uav_we_nei:
                    plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]
                else:
                    plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2]-1)]
        else:    	#飞机的高度为0或者1
            if plane.pos[2] ==1:
                plane.path = [(plane.pos[0], plane.pos[1], 0)]
                plane.remain_electricity = min(plane.charge+plane.remain_electricity, plane.capacity)
            else: 			#在停机坪上有两种
                if plane.remain_electricity < plane.capacity:
                    plane.path = [(plane.pos[0], plane.pos[1], 0)]
                    plane.remain_electricity += plane.charge
                else:  			#充满了，出发
                    plane.do_charge = "donot"

    else:
        z = min(map.high, map.low + 2)
        stop_pos = (map.parking_pos[0], map.parking_pos[1], z)

        plane_set_copy = plane_set.copy()
        for i in range(map.low, z-1):
            plane_set_copy[10000+i] = planetype()
            plane_set_copy[10000+i].pos = (map.parking_pos[0], map.parking_pos[1], i)

        uav_we_nei = ouruavneighbor(plane_set_copy, plane.pos)

        if plane.path == []:  		# 上次还没有规划，则本次规划，并进行异常处理；上次已经规划，但如果路径被挡，则重新规划，并进行规划异常处理
            if stop_pos in uav_enemy_nei or stop_pos in uav_we_nei:
                plane.path = [plane.pos]
            else:
                try:
                    camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set_copy, enemy_plane_set, role)
                    plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                except:
                    plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]
                    print("restruct_path error1")
        else:
            if plane.path[0] in uav_we_nei or plane.path[0] in uav_enemy_nei:
                if stop_pos in uav_enemy_nei or stop_pos in uav_we_nei:
                    plane.path = [plane.pos]
                else:
                    try:
                        camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set_copy, enemy_plane_set, role)
                        plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                    except:
                        plane.path.reverse()
                        pos = (plane.pos[0], plane.pos[1], plane.pos[2])
                        plane.path.append(pos)
                        plane.path.reverse()
                        print("restruct_path error2")

    plane.pos = plane.path[0]
    return plane


###########################################################################
'''定义运输机"trans"状态处理函数1，空载，转换货物目标，回去充电'''
###########################################################################
def handle_case1(plane, map, goods_avl, plane_set, enemy_plane_set, role, uav_enemy_nei): 			#plane是plane_set的某一个键的值，也为字典，类型为planetype
    test = (plane.z==0 and plane.do_charge == "do" ) or plane.do_charge=="donot"
    print("机号",plane.no, "test",test)
    if plane.x==map.parking_pos[0] and plane.y==map.parking_pos[1] and plane.z<map.low and test:    		#从停机坪出发
        plane = set_off(plane, map, plane_set, enemy_plane_set)

    else:
        plane.path.pop(0) #弹出路径的第一个
        plane.do_charge = "donot"
        print("选择前", "飞机号",plane.no, plane.pos, plane.target_goods_no, "可选货物", goods_avl)
        plane=selectgood(map, plane_set, enemy_plane_set, plane, goods_avl)     		#在外面的运输机只要空载就选择货物
        print("选择后", "飞机号",plane.no, plane.pos, plane.target_goods_no, plane.target_goods_pos)
        if plane.target_goods_no == []:      	 #货物没选到，如果在下面，先飞上来再说，在上面，电量不够，回去充电，电量够，再转转
            if plane.z < map.low and plane.x != map.parking_pos[0] and plane.y != map.parking_pos[1]:         #这里一般指运完货物之后
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                if (plane.pos[0], plane.pos[1], plane.pos[2] + 1) not in uav_we_nei and (plane.pos[0], plane.pos[1], plane.pos[2] + 1) not in uav_enemy_nei:
                    plane.path =[(plane.pos[0], plane.pos[1], plane.pos[2]+1)]
                else:
                    plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]

            else:      		#这一部分是没选到的飞机作随机游动（需要修改）
                if plane.remain_electricity < plane.capacity:
                    plane.do_charge = "do"   	 	#从这里调用go_home_charge
                    print("本飞机要回去充电，此时位置为", plane.pos)
                    plane = go_charge(plane, map, plane_set, enemy_plane_set, role, uav_enemy_nei)
                    print("充电规划为", plane.path)

                else:
                    uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                    results=map.neighbors(plane.pos, plane.pos)
                    buffer1=[]
                    for i in results:
                        if i not in uav_we_nei and i not in uav_enemy_nei:
                            buffer1.append(i)
                    results=buffer1
                    if results==[]:
                        plane.path = [plane.pos]
                    else:
                        plane.path =list(random.sample(results, 1))

                    plane.pos = plane.path[0]
                    #print("查看当前无人机邻居", uav_we_nei)
        else:       #选到货物
            if plane.z<=map.low and plane.x == plane.target_goods_pos[0] and plane.y == plane.target_goods_pos[1]:
                if  plane.z == 1:    #到达货物点上空
                    plane.goods_no =plane.target_goods_no         #给服务器发信息表示下一时刻到地面并同时拿货
                    plane.path=[(plane.target_goods_pos[0], plane.target_goods_pos[1], 0), (plane.target_goods_pos[0], plane.target_goods_pos[1], 1)]
                    plane.act = 2
                    plane.remain_electricity -=plane.target_goods_weight
                    print("取货点到了", "飞机地点", plane.pos, "飞机载货情况", plane.goods_no)
                else:
                    plane.path = [(plane.x, plane.y, plane.z-1)]

            else: 			#修改了货物目标
                if plane.change_target == "YES": 			#为什么要change目标，目标新生成，效益更高。
                    stop_pos = (plane.target_goods_pos[0], plane.target_goods_pos[1], map.low)
                    uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                    if stop_pos in uav_enemy_nei or stop_pos in uav_we_nei:
                        plane.path = [plane.pos]
                    else:
                        try:
                            camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, enemy_plane_set, role)
                            plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                        except:
                            plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]
                            print("restruct_path error3")
                            print("目标改变", "飞机号", plane.no, "货物号", plane.target_goods_no, "飞机当前位置", plane.pos, "路径",plane.path)
                else: 			#如果目标没变，还用以前的路径
                    uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                    #print(123456456, plane.path, uav_we_nei)
                    if plane.path == []:    	#这里是为了处理目标改变后路径规划发生异常时的情况
                        stop_pos = (plane.target_goods_pos[0], plane.target_goods_pos[1], map.low)
                        if stop_pos in uav_enemy_nei or stop_pos in uav_we_nei:
                            plane.path = [plane.pos]
                        else:
                            try:
                                camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, enemy_plane_set, role)
                                plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                            except:
                                plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]
                                print("restruct_path error4")
                    else:
                        if plane.path[0] in uav_we_nei or plane.path[0] in uav_enemy_nei:
                            stop_pos = (plane.target_goods_pos[0], plane.target_goods_pos[1], map.low)
                            if stop_pos in uav_enemy_nei or stop_pos in uav_we_nei:
                                plane.path = [plane.pos]
                            else:
                                try:
                                    camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, enemy_plane_set, role)
                                    plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                                except:
                                    plane.path.reverse()
                                    pos=(plane.pos[0], plane.pos[1], plane.pos[2])
                                    plane.path.append(pos)
                                    plane.path.reverse()
                                    print("restruct_path error5")

                    print("目标没变", "飞机号", plane.no,"货物号", plane.target_goods_no, plane.path)

    plane.pos = plane.path[0]
    print("无人机当前位置",plane.pos,(plane.x,plane.y,plane.z))
    return plane


###################################################################

'''定义状态处理函数2, 用于运货的路径规划，注意处理本架飞机后要修改本架飞机的plane.pos和plane.path，其他不变'''
def handle_case2(plane, map, plane_set, enemy_plane_set, role, uav_enemy_nei):
    plane.path.pop(0)         #弹出plane.path中的当前位置plane.pos
    uav_we_nei = ouruavneighbor(plane_set, plane.pos)     		#这里将取货飞机也分段，取货上升阶段，放货下降阶段
    if plane.z<map.low:           #低于最低飞行高度，两种情况，取完货和去放货
        if plane.x == plane.target_goods_desti[0] and plane.y == plane.target_goods_desti[1]:    #去放货（××××××××××××××××××××××××××××××××××××××××××××），先不管直接下去
            plane.path = [(plane.x, plane.y, plane.z -1)]
        else:   		 #在其他地方低于最低飞行高度，尽快上升
            if (plane.x, plane.y, plane.z+1) not in uav_we_nei and (plane.x, plane.y, plane.z+1) not in uav_enemy_nei:    #如果上方有我方飞机，则等待；否则上升
                plane.path =[(plane.x, plane.y, plane.z + 1)]
            else:
                plane.path =[(plane.x, plane.y, plane.z)]

    else:      	 #在空中正常运输
        if plane.z == map.low and  plane.x == plane.target_goods_desti[0] and plane.y == plane.target_goods_desti[1]:  #到达目的点上方map.low处
            plane.path = [(plane.x, plane.y, plane.z - 1)]
        else:
            if plane.path == []:        #弹出plane.path中的当前位置之后可能为空，常见于最低飞行高度处
                stop_pos=(plane.target_goods_desti[0], plane.target_goods_desti[1], map.low)
                if stop_pos in uav_we_nei  or stop_pos in uav_enemy_nei:
                    plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]
                else:
                    try:
                        camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, enemy_plane_set, role)
                        plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                    except:
                        plane.path =[(plane.pos[0], plane.pos[1], plane.pos[2])]

            else:   		#plane.path不为空
                if plane.path[0] in uav_we_nei or plane.path[0] in uav_enemy_nei:   		#下一步是障碍点，需要重新规划，否则什么都不做，照常
                    stop_pos = (plane.target_goods_desti[0], plane.target_goods_desti[1], map.low)
                    if stop_pos in uav_we_nei or stop_pos in uav_enemy_nei:
                        plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]
                    else:
                        try:
                            camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, enemy_plane_set, role)
                            plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                        except:
                            plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]

    print("路径",plane.path)
    plane.pos = plane.path[0]     		 #不管怎样，更新本机的位置，注意，本机的当前位置不更新，即plane.x, plane.y,plane.z不变
    plane.remain_electricity -= plane. target_goods_weight
    return plane


##################################################################################
'''
定义状态处理函数3，如何进行攻击
'''

##################################################################################
'''攻击敌人起始点，但不深入敌营'''
##################################################################################
def go_enemy_home(plane, map, plane_set):
    plane.path.pop(0)
    print("我的位置1",plane.path, plane.pos,plane.pre_path)
    if plane.x == map.parking_pos[0] and plane.y == map.parking_pos[1] and plane.z==map.low:
        plane.path = plane.pre_path
        uav_we_nei = ouruavneighbor(plane_set, plane.pos)
        if plane.path[0] in uav_we_nei:
            plane.path = [plane.pos] + plane.path
        print("我的位置2", plane.path)
    else:
        if plane.path ==[]:
            stop_pos = plane.pos  		 #敌机停机坪上方
            (x, y, z) = stop_pos
            results = [(x + 1, y - 1, z), (x + 1, y, z), (x + 1, y + 1, z), (x - 1, y + 1, z), (x, y + 1, z),(x - 1, y, z), (x - 1, y - 1, z), (x, y - 1, z)]
            results = filter(map.Inbound, results)
            results = filter(map.Outbuilding, results)
            uav_we_nei1 = []
            for i in plane_set:
                if plane_set[i].pos != plane.pos and Distance(plane.pos, plane_set[i].pos) <= 2:
                    uav_we_nei1.append(plane_set[i].pos)

            buffer = []
            for i in results:
                if i not in uav_we_nei1:
                    buffer.append(i)
            results = buffer

            if results == []:
                plane.path = [plane.pos]
            else:
                plane.path = [random.sample(results, 1)[0],stop_pos]  		# 注意，random.sample生成的是列表[(1,2),(3,4)] 这种类型

        else:
            if plane.pos not in plane.pre_path:
                uav_we_nei1 = []
                for i in plane_set:
                    if plane_set[i].pos != plane.pos and Distance(plane.pos, plane_set[i].pos) <= 2:
                        uav_we_nei1.append(plane_set[i].pos)
                if plane.path[0] in uav_we_nei1:
                    plane.path = [plane.pos] + plane.path
            else:
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                if plane.path[0] in uav_we_nei:
                    plane.path = [plane.pos] + plane.path

    plane.pos = plane.path[0]
    return plane


##################################################################################
'''选择被打负载飞机的函数，将已经被锁定的飞机排除出去，一旦锁定，plane.enemy_no为飞机号'''
##################################################################################
def selectenemyplane(map, plane_set, enemy_plane_set, all_enemy_plane, plane):
    print("有那些敌人", enemy_plane_set)
    for i in enemy_plane_set:
        print("敌人目的", enemy_plane_set[i].target_goods_desti)

    enemy_no_set=[]   #将所有被锁定的飞机记下
    for  i in plane_set:
        if plane_set[i].enemy_no !=[] and plane_set[i].enemy_no not in enemy_no_set:
            enemy_no_set.append(plane_set[i].enemy_no)

    print("被锁定的有", enemy_no_set)

    if plane.enemy_no == [] or all_enemy_plane[plane.enemy_no].status == 1:    			#如果没有目标敌机，或者目标敌机已经坠毁，则重新选择. 注意，就算锁定的这架飞机已经空载，仍然要撞他
        plane.enemy_no = []
        plane.enemy_desti=[]
        dist=10000;
        for i in enemy_plane_set:
            if enemy_plane_set[i].goods_no !=-1:   		#敌机中不是空载的
                if  enemy_plane_set[i].no not in enemy_no_set:   		#敌机没有被锁定，说明敌机现在可见
                    dist_enemy = Distance(enemy_plane_set[i].pos, enemy_plane_set[i].target_goods_desti)
                    dist_test = Distance(plane.pos, enemy_plane_set[i].target_goods_desti)
                    if dist>dist_test and dist_test < dist_enemy + 2*map.low:    		 #这里的目的地是在水平面上，但是这个碰撞关系仍然成立 dist_test-map.low< dist_enemy + map.low
                        dist=dist_test
                        plane.enemy_no=i
                        plane.enemy_desti=enemy_plane_set[i].target_goods_desti
        print("选择那架飞机", plane.enemy_no, plane.enemy_desti)

    return plane

###################################################################
'''路径规划到蹲目标点的函数'''
###################################################################
def go_enemy_desti(plane, map, plane_set, enemy_plane_set, all_ememy_plane, role):
    a= plane.enemy_desti
    stop_pos=(a[0], a[1], map.low)    #选到飞机之后设置路径规划目标点
    if plane.pos[0] == stop_pos[0] and plane.pos[1] == stop_pos[1] and plane.pos[2] <= stop_pos[2]:      		#在我们飞机飞到目标点上方，只要到上方就可以了
        if all_ememy_plane[plane.enemy_no].goods_no == -1 and all_ememy_plane[plane.enemy_no].z >= map.low:
            plane.enemy_no = []
            plane.enemy_desti = []
            plane.path = [(plane.x, plane.y, plane.z)]
        else:
            if (all_ememy_plane[plane.enemy_no].x == stop_pos[0] and all_ememy_plane[plane.enemy_no].y == stop_pos[1]  and all_ememy_plane[plane.enemy_no].z <=map.low) or plane.z<map.low:
                plane.path=[(plane.x, plane.y, plane.z-1)]
                if plane.z == 0:
                    plane.enemy_no = []
                    plane.enemy_desti = []
                    plane.path = [(plane.x, plane.y, plane.z+1)]
                    print("这个应该不会出现1")
            else:
                (x,y,z) = stop_pos
                results = [(x+1,y-1,z), (x+1,y,z), (x+1,y+1,z), (x-1,y+1,z), (x,y+1,z), (x-1,y,z), (x-1,y-1,z), (x,y-1,z)]
                results=filter(map.Inbound, results)
                results = filter(map.Outbuilding, results)
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                buffer= []
                for i in results:
                    if i not in uav_we_nei:
                        buffer.append(i)
                results = buffer

                if results == []:
                    plane.path = [plane.pos]
                else:
                    plane.path = [random.sample(results, 1)[0], stop_pos]    		#注意，random.sample生成的是列表[(1,2),(3,4)] 这种类型
    else:   		#我们的飞机没有到达敌方目的地上方的时候
        if plane.pos[2]<map.low:
            uav_we_nei = ouruavneighbor(plane_set, plane.pos)
            plane.path = [(plane.x, plane.y, plane.z + 1)]
            if plane.path[0] in uav_we_nei:
                plane.path =[(plane.x, plane.y, plane.z)]
        else:
            plane.path.pop(0)   		#注意
            print("什么鬼", plane.no, "wozaina", plane.pos, plane.path, "我要去哪", stop_pos)
            if plane.path ==[]:
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                print("我的邻居", uav_we_nei)
                if stop_pos in uav_we_nei:   		#自己人占了位置
                    plane.path =[(plane.x, plane.y, plane.z)]
                else:
                    try:
                        camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, enemy_plane_set, plane.role)
                        plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                    except:
                        plane.path = [(plane.x, plane.y, plane.z)]
            else:
                print("什么鬼",plane.no, "wozaina",plane.pos, plane.path, "我要去哪",stop_pos)
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                print("我的邻居", uav_we_nei)
                if plane.path[0] in uav_we_nei: 		#如果下一步有人
                    if stop_pos in uav_we_nei:   		#目标被自己人占了，停会儿
                        plane.path=[(plane.x, plane.y, plane.z)]
                    else:
                        try:
                            camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, enemy_plane_set, role)
                            plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                        except:
                            plane.path = [(plane.x, plane.y, plane.z)]

    plane.pos = plane.path[0]
    print("飞机位置", plane.no, plane.pos)
    return plane

###################################################################
'''没有选中飞机，那就攻击正在准备运货或者刚运完货的飞机吧'''
###################################################################
#基本逻辑：首先plane.enemy_no为空，设置一个plane.virtual_enemy_no, 发现附近
# 有低于map.low的空载飞机或者负载但是当前地点并不是货物的目的地点的飞机，则
# 进攻。一旦进攻失败，敌人溜走了，则将plane.virtual_enemy_no清空，再做随机游
# 荡。注意这个plane.enemy_no优先级大于plane.virtual_enemy_no，在攻击空载飞机
# 时，会被叫走打其他飞机

def attack_taking_goods(plane, map, plane_set, enemy_plane_set, all_ememy_plane,role, enemy_parking):

    enemy_no_set_v =[]
    for i in plane_set:
        if plane_set [i].enemy_no ==[] and plane_set[i].virtual_enemy_no != [] and plane_set[i].virtual_enemy_no not in enemy_no_set_v:
            enemy_no_set_v.append(plane_set[i].virtual_enemy_no)

    dist = 1000
    for i in enemy_plane_set:
        test = (enemy_plane_set[i].x, enemy_plane_set[i].y)!=(enemy_parking[0],enemy_parking[1])
        if enemy_plane_set[i].status ==0 and test:
            if 0<=enemy_plane_set[i].z< map.low and enemy_plane_set[i].goods_no ==-1: 		#正在运货
                if enemy_plane_set[i].no not in enemy_no_set_v:
                    stop_point = (enemy_plane_set[i].pos[0], enemy_plane_set[i].pos[1], map.low)
                    if dist>Distance (plane.pos, stop_point) and Distance (plane.pos, stop_point) <2*map.low:
                        dist = Distance (plane.pos, stop_point)
                        plane.virtual_enemy_no = i
                        plane.virtual_enemy_pos = (enemy_plane_set[i].pos[0], enemy_plane_set[i].pos[1],map.low)
            else:
                if 0<= enemy_plane_set[i].z< map.low and enemy_plane_set[i].goods_no !=-1 and (enemy_plane_set[i].target_goods_desti[0],enemy_plane_set[i].target_goods_desti[1]) != (enemy_plane_set[i].pos[0],enemy_plane_set[i].pos[1]):
                    if enemy_plane_set[i].no not in enemy_no_set_v:
                        stop_point = (enemy_plane_set[i].pos[0], enemy_plane_set[i].pos[1], map.low)
                        if dist > Distance(plane.pos, stop_point) and Distance(plane.pos, stop_point) < map.low:
                            dist = Distance(plane.pos, stop_point)
                            plane.virtual_enemy_no = i
                            plane.virtual_enemy_pos = (enemy_plane_set[i].pos[0], enemy_plane_set[i].pos[1], map.low)


    if plane.virtual_enemy_no != []:
        if plane.virtual_enemy_no not in enemy_plane_set:
            plane.virtual_enemy_no = []
            plane.virtual_enemy_pos = []
            plane.path = [plane.pos]
        else:
            if enemy_plane_set[plane.virtual_enemy_no].z >=map.low and (enemy_plane_set[plane.virtual_enemy_no].x, enemy_plane_set[plane.virtual_enemy_no].y) != (plane.virtual_enemy_pos[0], plane.virtual_enemy_pos[1]):
                plane.virtual_enemy_no=[]
                plane.virtual_enemy_pos = []
                plane.path = [plane.pos]
            else:
                if plane.pos[0] == plane.virtual_enemy_pos[0] and plane.pos[1] == plane.virtual_enemy_pos[1] and plane.pos[2] <= plane.virtual_enemy_pos[2]:
                    if plane.z==0:
                        plane.virtual_enemy_no = []
                        plane.virtual_enemy_pos=[]
                        plane.path=[(plane.x,plane.y,plane.z+1)]
                    else:
                        plane.path = [(plane.x,plane.y,plane.z-1)]
                else:
                    if plane.z <map.low:
                        uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                        plane.path = [(plane.x,plane.y,plane.z+1)]
                        if plane.path[0] in uav_we_nei:
                            plane.path =[plane.pos]

                    else:
                        plane.path.pop(0)
                        if plane.path ==[]:
                            uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                            if plane.virtual_enemy_pos in uav_we_nei:  			# 自己人占了位置
                                plane.path = [(plane.x, plane.y, plane.z)]
                            else:
                                try:
                                    camefrom = AstarSearch(map, plane.pos, plane.virtual_enemy_pos, plane_set, enemy_plane_set, plane.role)
                                    plane.path = restruct_path(camefrom, plane.pos, plane.virtual_enemy_pos)
                                except:
                                    plane.path = [(plane.x, plane.y, plane.z)]
                        else:
                            uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                            if plane.path[0] in uav_we_nei:  		# 如果下一步有人
                                if plane.virtual_enemy_pos in uav_we_nei:  		# 目标被自己人占了，停会儿
                                    plane.path = [(plane.x, plane.y, plane.z)]
                                else:
                                    try:
                                        camefrom = AstarSearch(map, plane.pos, plane.virtual_enemy_pos, plane_set, enemy_plane_set, role)
                                        plane.path = restruct_path(camefrom, plane.pos, plane.virtual_enemy_pos)
                                    except:
                                        plane.path = [(plane.x, plane.y, plane.z)]

        plane.pos = plane.path[0]
    return plane


###################################################################
'''连运货飞机都没选中，那就去选贵的货物吧'''
###################################################################
#基本逻辑：

def attack_goods(plane, map, goods_avl, plane_set,role):
    #先选货物
    for i in plane_set:
        if plane_set[i].goods_no !=-1 and plane_set[i].goods_no in goods_avl:
            del goods_avl[plane_set[i].goods_no]
        if plane_set[i].target_goods_no !=[] and plane_set[i].target_goods_no in goods_avl:
            del goods_avl[plane_set[i].target_goods_no]
        if plane_set[i].virtual_goods_no !=[] and plane_set[i].virtual_goods_no in goods_avl:
            del goods_avl[plane_set[i].virtual_goods_no]

    lamb1=0.4
    lamb2=0.1
    previous_goods_no = []
    if plane.virtual_goods_no == [] or plane.virtual_goods_no not in goods_avl:
        choice_para = 0
        plane.virtual_goods_no == []
    else:
        previous_goods_no = plane.virtual_goods_no
        plane.virtual_goods_no=[]
        dis = lamb1 * Distance(plane.pos, plane.target_goods_pos) + lamb2 * Distance(plane.target_goods_pos, plane.target_goods_desti)
        choice_para = goods_avl[previous_goods_no].value / (dis ** 2)

    for i in goods_avl:  		# i为键，表示货物编号，其值为字典
        if  Distance(plane.pos, goods_avl[i].start_pos) <= goods_avl[i].left_time:
            a = lamb1 * Distance(plane.pos, goods_avl[i].start_pos) + lamb2 * Distance(goods_avl[i].start_pos,goods_avl[i].end_pos)
            b = goods_avl[i].value / (a ** 2)
            c = (goods_avl[i].start_x, goods_avl[i].start_y)

            if choice_para <= b:
                choice_para = b
                plane.virtual_goods_no = goods_avl[i].no
                plane.virtual_goods_pos = (goods_avl[i].start_x, goods_avl[i].start_y, map.low)

    if plane.virtual_goods_no== previous_goods_no:
        plane.change_target = "NO"
    else:
        plane.change_target = "YES"  		# 表示修改货物目标，需要重新规划

    # 再到货物目的地

    if plane.virtual_goods_no !=[]:
        (x,y,z) = plane.virtual_goods_pos
        results = [(x-2, y - 2, z), (x-2,y-1,z),(x-2,y,z),(x-2,y+1,z),(x-2,y+2,z),(x-1,y-2,z),(x-1,y+2,z),(x,y-2,z),(x,y+2,z),(x+1,y-2,z),(x+1,y+2,z),(x+2, y - 2, z), (x+2,y-1,z),(x+2,y,z),(x+2,y+1,z),(x+2,y+2,z)]
        results = filter(map.Inbound, results)
        results = filter(map.Outbuilding, results)
        uav_we_nei = ouruavneighbor(plane_set, plane.pos)
        buffer = []
        for i in results:
            if i not in uav_we_nei:
                buffer.append(i)
        results = buffer

        if results == []:
            stop_pos = (x,y,z)
        else:
            stop_pos =random.sample(results, 1)[0]

        if plane.pos == plane.virtual_goods_pos:
            plane.path = [plane.pos]
        else:
            if plane.change_target == "YES":  		# 为什么要change目标，目标新生成，效益更高。
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                if stop_pos in uav_we_nei:
                    plane.path = [plane.pos]
                else:
                    try:
                        camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, {}, role)
                        plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                        if plane.path ==[]:
                            plane.path=[plane.pos]
                    except:
                        plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]
            else:  			# 如果目标没变，还用以前的路径
                uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                if plane.path == []:  			# 这里是为了处理目标改变后路径规划发生异常时的情况
                    stop_pos = (plane.target_goods_pos[0], plane.target_goods_pos[1], map.low)
                    if  stop_pos in uav_we_nei:
                        plane.path = [plane.pos]
                    else:
                        try:
                            camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, {}, role)
                            plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                        except:
                            plane.path = [(plane.pos[0], plane.pos[1], plane.pos[2])]
                            print("restruct_path error40")
                else:
                    if plane.path[0] in uav_we_nei:
                        stop_pos = (plane.target_goods_pos[0], plane.target_goods_pos[1], map.low)
                        if stop_pos in uav_we_nei:
                            plane.path = [plane.pos]
                        else:
                            try:
                                camefrom = AstarSearch(map, plane.pos, stop_pos, plane_set, {}, role)
                                plane.path = restruct_path(camefrom, plane.pos, stop_pos)
                            except:
                                plane.path.reverse()
                                pos = (plane.pos[0], plane.pos[1], plane.pos[2])
                                plane.path.append(pos)
                                plane.path.reverse()
                                print("restruct_path error50")

        print("第",plane.no,"架飞机，所选货物", plane.virtual_goods_no, "目标是否改变", plane.change_target, plane.virtual_goods_pos, "1231552",plane.pos,plane.path)
        plane.pos = plane.path[0]
    return plane

###################################################################
'''处理战斗机的规划问题'''
###################################################################

def handle_case3(plane, map, goods_avl, plane_set, enemy_plane_set, all_ememy_plane, role,enemy_parking):

    if plane.x == map.parking_pos[0] and plane.y == map.parking_pos[1] and 0<=plane.z<map.low:
        plane = set_off(plane, map, plane_set, enemy_plane_set)
    else:
        if plane.attack_home == "attack":    		#攻击敌人家的函数
            plane=go_enemy_home(plane, map, plane_set)
        else:
            plane = selectenemyplane(map, plane_set, enemy_plane_set, all_ememy_plane, plane)
            if plane.enemy_no != []:
                plane = go_enemy_desti(plane, map, plane_set, enemy_plane_set, all_ememy_plane, role)
            else:
                print("前，第", plane.no, "架飞机，所选飞机和位置", plane.virtual_enemy_no, plane.virtual_enemy_pos, "飞机位置和路径", plane.pos, plane.path)
                plane = attack_taking_goods(plane, map, plane_set, enemy_plane_set, all_ememy_plane, role, enemy_parking)
                print("后，第", plane.no, "架飞机，所选飞机和位置", plane.virtual_enemy_no, plane.virtual_enemy_pos, "飞机位置和路径",plane.pos, plane.path)
                if plane.virtual_enemy_no ==[]:
                    print("前，第", plane.no, "架飞机，所选货物和位置", plane.virtual_goods_no, plane.virtual_goods_pos, "飞机位置和路径",plane.pos, plane.path)
                    plane=attack_goods(plane, map, goods_avl, plane_set, role)
                    print("后，第", plane.no, "架飞机，所选货物和位置", plane.virtual_goods_no, plane.virtual_goods_pos, "飞机位置和路径",
                          plane.pos, plane.path)
                    if plane. virtual_goods_no ==[]:
                        uav_we_nei = ouruavneighbor(plane_set, plane.pos)
                        results = map.neighbors(plane.pos, plane.pos)
                        buffer1 = []
                        for i in results:
                            if i not in uav_we_nei:
                                buffer1.append(i)
                        results = buffer1
                        if results == []:
                            plane.path = [plane.pos]
                        else:
                            plane.path = list(random.sample(results, 1))

                    plane.pos = plane.path[0]
    return plane





###########################################################################

'''用户自定义函数, 返回字典FlyPlane, 需要包括 "UAV_info", "purchase_UAV" 两个key.'''
def AlgorithmCalculationFun(Map, Match_Status, pstFlayPlane, map, plane_set, all_enemy_plane, cheapest_type,time,sort_plane, enemy_parking, attack_home_path,change_time):
    good_list={}
    for i in Match_Status["goods"]:
        good_list[i["no"]]=goodsInfo(i)

    goods_avl = {}
    goods_avl = get_goods_avl(Match_Status)
    plane_set = get_planes_set(Match_Status, Map, plane_set, cheapest_type, time, enemy_parking, attack_home_path, change_time)
    enemy_plane_set=get_enemy_plane(Match_Status, Map, cheapest_type,time,change_time)
    all_enemy_plane =  get_all_enemy_plane(Match_Status, Map, all_enemy_plane, cheapest_type, time,change_time)



    print("比赛信息", Match_Status["UAV_we"])
    #print("获取后飞机信息", "No",plane_set[0].no, "pos",plane_set[0].pos, "status",plane_set[0].status, "目标货物号",plane_set[0].target_goods_no, "act", plane_set[0].act, plane_set[1].no, plane_set[1].target_goods_no, plane_set[1].act)
    #print("飞机排序前的no", sort_plane)
    sort_plane = sortplane(plane_set, map,sort_plane)

    #print("飞机排序后的no",sort_plane)
    for i in sort_plane:      		#每一架飞机的决策行动，最终对plane_set中的参数做修改
        if plane_set[i].act == -1:  		#坠毁
            print("坠机id",i)
        elif plane_set[i].act == 1:
            print("第", i, "架",plane_set[i].role, "飞机规划前位置", plane_set[i].pos,"采取行动",plane_set[i].act, "充电否", plane_set[i].do_charge)
            uav_enemy_nei = enemyuavneighbor(enemy_plane_set, plane_set[i].pos, map)
            plane_set[i] = handle_case1(plane_set[i], map, goods_avl, plane_set,enemy_plane_set, plane_set[i].role,uav_enemy_nei)
            print("第", i, "架",plane_set[i].role, "飞机规划后位置", plane_set[i].pos)

        elif plane_set[i].act == 2:
            print("第", i, "架",plane_set[i].role, "飞机规划前位置",plane_set[i].pos, "采取行动",plane_set[i].act, "充电否", plane_set[i].do_charge)
            uav_enemy_nei = enemyuavneighbor(enemy_plane_set, plane_set[i].pos, map)
            plane_set[i] = handle_case2(plane_set[i], map, plane_set, enemy_plane_set, plane_set[i].role, uav_enemy_nei)
            print("第", i, "架",plane_set[i].role, "飞机规划后位置", plane_set[i].pos,"飞机价值", plane_set[i].plane_value, "飞机总值", plane_set[i].all_value, plane_set[i].path)
        elif plane_set[i].act == 3:
            print("第", i, "架", plane_set[i].role, "选到人了吗", plane_set[i].enemy_no, "敌人目标点", plane_set[i].enemy_desti, "wodeweizhihelujing", plane_set[i].pos, plane_set[i].path)
            goods_avl2 = {}
            goods_avl2= get_goods_avl(Match_Status)
            plane_set[i]=handle_case3(plane_set[i], map, goods_avl2, plane_set, enemy_plane_set, all_enemy_plane,plane_set[i].role,enemy_parking)
            print("第", i, "架",plane_set[i].goods_no, plane_set[i].path,"虚拟货物",plane_set[i].virtual_goods_no)


#    print(987654321, plane_set[0].target_goods_no, plane_set[1].target_goods_no)

    current_status=[]
    for uav in Match_Status["UAV_we"]:
        U={}
        for i in plane_set:
            if  plane_set[i].path != [] and plane_set[i].no == uav["no"] and plane_set[i].status != 1 :
                #print("飞机的碰撞情况",plane_set[i].status)
                U["no"]= plane_set[i].no
                U["x"]=plane_set[i].path[0][0]
                U["y"]=plane_set[i].path[0][1]
                U["z"]=plane_set[i].path[0][2]
                U["goods_no"] = plane_set[i].goods_no
                U["remain_electricity"] = plane_set[i].remain_electricity
                current_status.append(U)
                break

    FlyPlane = current_status
    return plane_set, enemy_plane_set, all_enemy_plane, FlyPlane, sort_plane








def main(szIp, nPort, szToken):
    print("server ip %s, prot %d, token %s\n", szIp, nPort, szToken)

    #Need Test // 开始连接服务器
    hSocket = socket.socket()

    hSocket.connect((szIp, nPort))

    #接受数据  连接成功后，Judger会返回一条消息：
    nRet, _ = RecvJuderData(hSocket)
    if (nRet != 0):
        return nRet
    

    # // 生成表明身份的json
    token = {}
    token['token'] = szToken        
    token['action'] = "sendtoken"   

    
    #// 选手向裁判服务器表明身份(Player -> Judger)
    nRet = SendJuderData(hSocket, token)
    if nRet != 0:
        return nRet

    #//身份验证结果(Judger -> Player), 返回字典Message
    nRet, Message = RecvJuderData(hSocket)
    if nRet != 0:
        return nRet
    
    if Message["result"] != 0:
        print("token check error\n")
        return -1

    # // 选手向裁判服务器表明自己已准备就绪(Player -> Judger)
    stReady = {}
    stReady['token'] = szToken
    stReady['action'] = "ready"

    nRet = SendJuderData(hSocket, stReady)
    if nRet != 0:
        return nRet

    # 对战开始通知(Judger -> Player)，这里的Message表示无人机收到的信息
    nRet, Message = RecvJuderData(hSocket)
    if nRet != 0:
        return nRet
    
    #初始化地图信息
    pstMapInfo = Message["map"]  
    
    #初始化比赛状态信息
    pstMatchStatus = {}
    pstMatchStatus["time"] = 0

    #初始化飞机状态信息
    pstFlayPlane = {}
    pstFlayPlane["nUavNum"] = len(pstMapInfo["init_UAV"])
    pstFlayPlane["astUav"] = []

    #每一步的飞行计划
    FlyPlane_send = {}
    FlyPlane_send["token"] = szToken
    FlyPlane_send["action"] = "flyPlane"

    for i in range(pstFlayPlane["nUavNum"]):
        pstFlayPlane["astUav"].append(pstMapInfo["init_UAV"][i])

    # // 根据服务器指令，不停的接受发送数据
    ##########################################################
    ##########################################################
    plane_set={}
    enemy_plane_set = {}
    all_enemy_plane={}
    map=[]
    uav_dict = {}
    BuyUav=-1
    max_time=0

    while True:
        timestart = time.time()
        if pstMatchStatus["time"] == 0:
            sort_plane = []
            FlyPlane = pstFlayPlane["astUav"]
            map = MAP(pstMapInfo)
            for i in pstMapInfo["UAV_price"]:
                uav_dict[i["type"]] = uavlist(i)

            sort_ratio_uav =sortuavratio(uav_dict)
            cheapest_type, cheapest_value=lessvalue(uav_dict)
            print("便宜货飞机",cheapest_type, cheapest_value)

            for i in pstMapInfo["init_UAV"]:
                sort_plane.append(i["no"])
            print("飞机排序",sort_plane)
            enemy_parking=[]
            attack_home_path=[]

        else:
            if pstMatchStatus["time"] == 1:
                for i in pstMatchStatus["UAV_enemy"]:
                    if i["x"] >=0 and i["y"]>=0 and i["z"] == 0:
                        enemy_parking = (i["x"],  i["y"], 0)
                        break

                if (Distance(enemy_parking, map.parking_pos))<= len(pstMatchStatus["UAV_we"]):
                    change_time = 1
                else:
                    change_time=50
                print(enemy_parking, change_time,map.parking_pos, len(plane_set))

                we_home = (map.parking_pos[0], map.parking_pos[1], map.low)
                we_middle_enemy = (enemy_parking[0], enemy_parking[1], map.low)
                camefrom = AstarSearch(map, we_home, we_middle_enemy, {}, {}, "fighting")
                path1 = restruct_path(camefrom, we_home, we_middle_enemy)
                attack_home_path = path1

                plane_set, enemy_plane_set, all_enemy_plane, FlyPlane, sort_plane = AlgorithmCalculationFun(pstMapInfo, pstMatchStatus, pstFlayPlane, map, plane_set,  all_enemy_plane, cheapest_type, pstMatchStatus["time"], sort_plane, enemy_parking, attack_home_path, change_time)

                print("飞机排序1", pstMatchStatus["time"], sort_plane, attack_home_path)
                enemy_num= get_enemy_num(pstMatchStatus)
            else:
                print("飞机排序0000", pstMatchStatus["time"], sort_plane, attack_home_path)
                plane_set, enemy_plane_set, all_enemy_plane, FlyPlane, sort_plane = AlgorithmCalculationFun(pstMapInfo,  pstMatchStatus, pstFlayPlane,map,plane_set, all_enemy_plane, cheapest_type, pstMatchStatus["time"], sort_plane, enemy_parking, attack_home_path,change_time)
                print("飞机排序1111", pstMatchStatus["time"], sort_plane, attack_home_path)
                enemy_num = get_enemy_num(pstMatchStatus)

            BuyUav = purchase(sort_ratio_uav, cheapest_type, cheapest_value, uav_dict, pstMapInfo, pstMatchStatus, plane_set, enemy_plane_set, enemy_num)

        FlyPlane_send['UAV_info'] = FlyPlane

        if BuyUav != []:
                FlyPlane_send['purchase_UAV'] = BuyUav
        else:
              FlyPlane_send['purchase_UAV'] = []

        timeend=time.time()
        if  timeend-timestart > max_time:
            max_time=timeend-timestart

        print("耗时，最大耗时", timeend-timestart, max_time)

        print("发给服务器的数据",plane_set, pstMatchStatus["time"], FlyPlane_send['UAV_info'], BuyUav)


   ##########################################################
   #########################################################



        # // 进行当前时刻的数据计算, 填充飞行计划，注意：1时刻不能进行移动，即第一次进入该循环时


        print(pstMatchStatus["time"])
        # //发送飞行计划
        nRet = SendJuderData(hSocket, FlyPlane_send)
        if nRet != 0:
            return nRet
        
        # // 接受当前比赛状态
        nRet, pstMatchStatus = RecvJuderData(hSocket)
        if nRet != 0:
            return nRet
        
        if pstMatchStatus["match_status"] == 1:
            print("game over, we value %d, enemy value %d\n", pstMatchStatus["we_value"], pstMatchStatus["enemy_value"])
            hSocket.close()
            return 0

if __name__ == "__main__":
    if len(sys.argv) == 4:
        print("Server Host: " + sys.argv[1])
        print("Server Port: " + sys.argv[2])
        print("Auth Token: " + sys.argv[3])
        main(sys.argv[1], int(sys.argv[2]), sys.argv[3])
    else:
        print("need 3 arguments")