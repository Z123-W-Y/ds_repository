from machine import Pin
import sensor, image, time, pyb, ulab as np
#import seekfree
from pyb import UART
import ustruct
import math
from pyb import LED
# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # 设置图像色彩格式为RGB565格式
sensor.set_framesize(sensor.QVGA)  # 设置图像大小为160*120
#sensor.set_windowing((63,17,165,143))
sensor.set_auto_whitebal(True)      # 设置自动白平衡
sensor.set_brightness(2000)         # 设置亮度为3000
sensor.skip_frames(time = 20)       # 跳过帧
uart = UART(3, 115200) #配置串口
uart.init(115200,8,None,1) #8个数据位，无奇偶校验，1个停止位
clock = time.clock()
red = (0, 100, 12, 127, -128, 127)#
green =(86, 100, -128, -6, -128, 24)
red1=(4, 100, -128, 127, -128, 127)#二值化之后白色的阈值（也是二值化之后红色激光的阈值）
#串口发送代码
#sign：模式切换
def sending_data(sign,cx,cy):
    global uart;
    data = ustruct.pack("<bbhhb",      #格式为俩个字符俩个短整型(2字节)
                   0x55,
                   sign,
                   int (cx),
                   int (cy),
                   0xAA)
    uart.write(data);   #必须要传入一个字节数组
#自定义清空文件内容函数
def clear_x():
    file = open("x.txt", "w")
    # 清空文件
    file.write("")
    file.close()
def clear_y():
    file = open("y.txt", "w")
    # 清空文件
    file.write("")
    file.close()
#读取文件内容
def read_txt(txt):
    numbers = []
    with open(txt, 'r') as test:
        lines = test.readlines()
        for line in lines:
            line = float(line.strip('\n'))   #将\n去掉
            numbers.append(int(line))   #将空格作为分隔符将一个字符切割成一个字符数组
    return numbers
mode=1#识别方框切换
mode_fk=1
j=0#四个顶点切换
j_x=0#记录一条边经过的点数
error_x=0
error_y=0
r=0
idx=1#当idx=1时计算一次直线方程
green_idx=1
m=0#斜率
b=0#截距
data=ord("1")#保存接收数据
data_id=1
n_x_red=0#保存当前红点的x轴
n_y_red=0#当前红点的y轴
t_x_red=[]#记录红点走过的点的x轴
t_y_red=[]#记录红点走过的点的y轴
g_x_red=0#保存满足绿点跟踪红点走过的点的x轴
g_y_red=0#保存满足绿点跟踪红点走过的点的y轴
find_blobs_roi=(115,30,185,184)
#读取文件中的四个角的x，y的值还有中心点的坐标
def read_xy():
    x=[]
    y=[]
    try:
        number_x=read_txt("x.txt")
        number_y=read_txt("y.txt")
        for index, number in enumerate(number_x):
            if index >= 4:  # 遍历前四行
                break
            x.append(number)
        x.append(x[0])
        for index, number in enumerate(number_y):
            if index >= 4:  # 遍历前四行
                break
            y.append(number)
        y.append(y[0])
        x_central=number_x[4]
        y_central=number_y[4]
        print(x)
        print(y)
        print(x_central)
        print(y_central)
    except:
        x=[140,267,262,137,140]
        y=[46,52,178,172,46]
        x_central=208#方框中心x轴的坐标
        y_central=115#方框中心y轴的坐标
    return x, y, x_central, y_central
#初始化四个角点还有中心点的坐标
x,y,x_central,y_central=read_xy()
x_green=0
y_green=0
sign=0#记录读取第几个点
green_sign=0#第二个激光启用寻找绿点和红点标志位
while(True):
    clock.tick()
    img = sensor.snapshot()
    LED(1).off()
    #接收uart数据
    if uart.any():
        data=uart.readchar()
        if data==ord('B'):
            x,y,x_central, y_central=read_xy()
#*******************************方框校准，存放到txt文件********************************
    if data==ord("Z") and sign<5:#sign模式题号
        if sign==0:
            clear_x()
            clear_y()
            x=[]
            y=[]
        img.binary([red])#二值化处理
        img.dilate(1)#膨胀算法
        red_blobs = img.find_blobs([red1])#寻找色块
        for blob in red_blobs:#找到色块返回xy的坐标
            x_red=blob.cx()
            y_red=blob.cy()
            img.draw_circle(blob.cx(), blob.cy(), 4, color = (255, 0, 0))#在找到的该中心点画一个半径为4的圆
        sending_data(0x02,sign,0)#题号，第几个点
        with open(r'y.txt','a+') as test_y:
            test_y.write(str(y_red)+'\n')
        with open(r'x.txt','a+') as test_x:
            test_x.write(str(x_red)+'\n')
        sign=sign+1
        data=1
        LED(1).on()
        if sign==5:
            number_x=read_txt("x.txt")
            number_y=read_txt("y.txt")
            for index, number in enumerate(number_x):
                if index >= 4:  # 遍历前四行
                    break
                x.append(number)
            x.append(x[0])
            for index, number in enumerate(number_y):
                if index >= 4:  # 遍历前四行
                    break
                y.append(number)
            y.append(y[0])
            x_central=number_x[4]
            y_central=number_y[4]
            print(x)
            print(y)
            print(x_central)
            print(y_central)
    #######################发挥题，绿追红##############################
    if data==ord('E'):
        if green_sign==0:
            img.binary([green])
            img.dilate(2)
            green_blobs = img.find_blobs([red1],roi=[112,20,154,149])
            if(len(green_blobs)==0):
                sending_data(0x01,1000,1000)
            for blob in green_blobs:
                x_green=blob.cx()
                y_green=blob.cy()
                img.draw_circle(blob.cx(), blob.cy(), 4, color = (0,0,255))
                green_sign=1
        if (green_sign==1):
            red_n=0
            img.binary([red])#二值化处理
            img.dilate(2)#膨胀算法
            red_blobs = img.find_blobs([red1],roi=[112,20,154,149] )#寻找色块
            if(len(red_blobs)==0):
                sending_data(0x01,1000,1000)
            for blob in red_blobs:
                x_red=blob.cx()
                y_red=blob.cy()
                img.draw_circle(x_green, y_green, 4, color = (0, 255, 255))
                img.draw_circle(x_red, y_red, 4, color = (255,0,0))
                error_x=x_green-x_red
                error_y=y_green-y_red
                sending_data(0x01,error_x,error_y)
#############第一题定点##########################
    if data==ord('A'):
        img.binary([red])
        img.dilate(2)
        red_blobs = img.find_blobs([red1],roi=find_blobs_roi)
        if(len(red_blobs)==0):
            sending_data(0x01,1000,1000)
            print(0,0)
        for blob in red_blobs:
            x_red=blob.cx()
            y_red=blob.cy()
            img.draw_circle(blob.cx(), blob.cy(), 4, color = (255, 0, 0))
            img.draw_circle(x_central, y_central, 4, color = (0, 255, 255))
            error_x=x_central-x_red
            error_y=y_central-y_red
#                mode1=1
            sending_data(0x01,error_x,error_y)
            print(error_x,error_y)
    #####################第二题外方框###########################
    if data==ord('B') and data_id==1:
        data_id=0
    #####################寻找方框更改红点阈值#######################
    if data==ord('C'):
        red = (0, 100, 10, 127, -128, 127)
    #####################第二题和第三题#############################
    if data==ord('C') or data==ord('B') or data==ord('D'):
        ####################################当是第三题时进行寻找方框###########################
        if mode==1 and (data==ord('C') or data==ord('D')):
            for r in img.find_rects(threshold = 10000):
                # 判断矩形边长是否符合要求

                img.draw_rectangle(r.rect(), color = (255, 0, 0), scale = 4)
                #限制方框大小
                if (r.w() > 50 and r.w()<100) and (r.h() > 40 and r.h()<100):
                    # 在屏幕上框出矩形
                    print("r.h()",r.h(),"r.w",r.w())
                    img.draw_rectangle(r.rect(), color = (255, 0, 0), scale = 4)
                    # 获取矩形角点位置
                    corner = r.corners()
                    # 在屏幕上圈出矩形角点
                    cx11=(int)(corner[0][0])
                    cy11=(int)(corner[0][1])
                    cx22=(int)(corner[1][0])
                    cy22=(int)(corner[1][1])
                    cx33=(int)(corner[2][0])
                    cy33=(int)(corner[2][1])
                    cx44=(int)(corner[3][0])
                    cy44=(int)(corner[3][1])
                    #计算方框两斜对角顶点的斜率和截距为后面收缩顶点坐标做准备
                    m1=(cy33-cy11)/(cx33-cx11)
                    b1=cy11-m1*cx11
                    m2=(cy44-cy22)/(cx44-cx22)
                    b2=cy22-m2*cx22
                    print("m",m1)
                    print("m",m2)
                    if(data==ord('C')):
                        cx1=cx11+2
                        cy1=cy11-2
                        cx2=cx22-2
                        cy2=cy22-2
                        cx3=cx33-2
                        cy3=cy33+2
                        cx4=cx44+2
                        cy4=cy44+2
                    #判断他们的斜率从而判断是先从x轴还是y轴先开始改变
                    elif(data==ord('D')):
                        if abs(m1)>1:
                            cy1=int(cy11-2)
                            cx1=int((cy1-b1)/m1)
                            cy3=int(cy33+2)
                            cx3=int((cy3-b1)/m1)
                        elif abs(m1)<=1:
                            if cx33>cx11:
                                cx1=int(cx11+1)
                                cy1=int(m1*cx1+b1)
                                cx3=int(cx33-1)
                                cy3=int(m1*cx3+b1)
                            else:
                                cx1=int(cx11-1)
                                cy1=int(m1*cx1+b1)
                                cx3=int(cx33+1)
                                cy3=int(m1*cx3+b1)
                        if abs(m2)>1:
                            cy2=int(cy22-2)
                            cx2=int((cy2-b2)/m2)
                            cy4=int(cy44+2)
                            cx4=int((cy4-b2)/m2)
                        elif abs(m2)<=1:
                            if cx22>cx44:
                                cx2=int(cx22-1)
                                cy2=int(m2*cx2+b2)
                                cx4=int(cx44+2)
                                cy4=int(m2*cx4+b2)
                            else:
                                cx2=int(cx22+2)
                                cy2=int(m2*cx2+b2)
                                cx4=int(cx44-2)
                                cy4=int(m2*cx4+b2)
            #串口通信传输的数据
                    mode=2#使mode变为2跳出寻找方框进行接下来的操作
                    x=[cx1,cx2,cx3,cx4,cx1]
                    y=[cy1,cy2,cy3,cy4,cy1]
        if mode==2 or data==ord('B'):
            red_blobs = img.find_blobs([red],roi=find_blobs_roi)
            img.draw_circle(x[0], y[0], 1, color = (0, 255,255))#圈出各个角点坐标
            img.draw_circle(x[1], y[1], 1, color = (0, 0, 255))
            img.draw_circle(x[2], y[2], 1, color = (0, 0, 255))
            img.draw_circle(x[3], y[3], 1, color = (0, 0, 255))
            if(len(red_blobs)==0):
                sending_data(0x01,1000,1000)
            for blob in red_blobs:
                x_red=blob.cx()
                y_red=blob.cy()
                img.draw_circle(blob.cx(), blob.cy(), 4, color = (255, 0, 0))
                #先让红点到矩形的第一个角点
                if(mode_fk==1):
                    error_x=x[0]-x_red
                    error_y=y[0]-y_red
                    r=(x_red-x[0])**2+(y_red-y[0])**2
                    if r<5:
                        mode_fk=2
                        x_d=x[0]
                        y_d=y[0]
                #然后进行下一步
                if mode_fk==2:
                    #计算出斜率和截距当前点和下一个点的斜率和截距
                    if ((x[j+1]-x[j])==0):
                        m=(y[j+1]-y[j])/0.0001
                    else:
                        m=(y[j+1]-y[j])/(x[j+1]-x[j])
                    b=y[j]-m*x[j]
                    #当是第二题时分成15段
                    if data==ord('B'):
                        Duan=2
                    #当是第三题按长短来分段
                    if data==ord('D') or data==ord('C'):
                        if math.sqrt((x[j+1]-x[j])**2+(y[j+1]-y[j])**2)>70 and math.sqrt((x[j+1]-x[j])**2+(y[j+1]-y[j])**2)<=85:
                            Duan=4
                        elif math.sqrt((x[j+1]-x[j])**2+(y[j+1]-y[j])**2)<70:
                            Duan=2
                    #判断斜率是否大于1
                    if abs(m)<=1 and idx==1:
                        d=(x[j+1]-x[j])/Duan
                        x_d=int(x_d+d)
                        y_d=int(m*x_d+b)
                        idx=0
                        j_x+=1
                    elif abs(m)>1 and idx==1 :
                        d=(y[j+1]-y[j])/Duan
                        y_d=int(y_d+d)
                        x_d=int((y_d-b)/m)
                        idx=0
                        j_x+=1
                    error_x=x_d-x_red
                    error_y=y_d-y_red
                    r=(x_red-x_d)**2+(y_red-y_d)**2
                #每次当距离小于5时，使idx=1确定下一个点的坐标
                    if r<=2:
                        idx=1
                    #每当一条边识别了5个点时，进行下一条边的计算
                    if j_x>=Duan:
                        j=j+1
                        j_x=0
                        x_d=x[j]
                        y_d=y[j]
                    #当四条边都识别完了，进行下一个阶段
                    if j==4:
                        mode_fk=3
                        j_x=0
                        j=0
                    img.draw_circle(x_d, y_d, 2, color = (0, 0, 255))
                #最后定点到第一个顶点
                if mode_fk==3:
                    error_x=x[0]-x_red
                    error_y=y[0]-y_red
                    r=(x_red-x[0])**2+(y_red-y[0])**2
                    if r<=2:
                        mode_fk=1
                        error_x=0
                        error_y=0
                        data=1
                        mode=1
                sending_data(0x01,error_x,error_y)
