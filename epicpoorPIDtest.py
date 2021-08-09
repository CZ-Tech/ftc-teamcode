import turtle
t=turtle.Pen()
t.goto(-400,-200)
t.clear()

nowvelocity=0
goalvelocity=3000
lateP=goalvelocity
needvelocity=0
P=0
I=0
D=0
Kp=0.2# //比例增益 //TODO 待调整
Ki=0.15# //积分时间常数 //TODO 待调整
Kd=0.03# //微分时间常数 //TODO 待调整
for i in range(0,50):
    

    P=goalvelocity-nowvelocity
    
    I=I+P
    D=(P-lateP)



    needvelocity=Kp*P+Ki*I+Kd*D

    lateP=P


    print (nowvelocity)
#测试用数据
    t.goto(10*i-400,nowvelocity/6.5-200)
    nowvelocity=needvelocity


    
#外力变化
nowvelocity=2000
#外力变化

for i in range(50,100):
    

    P=goalvelocity-nowvelocity
    
    I=I+P
    D=(P-lateP)



    needvelocity=Kp*P+Ki*I+Kd*D

    lateP=P


    print (nowvelocity)
#测试用数据
    t.goto(10*i-400,nowvelocity/6.5-200)
    nowvelocity=needvelocity


