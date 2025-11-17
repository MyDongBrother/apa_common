import math
import matplotlib.pyplot as plt

def sign(value):
    if value > 0.001:
        return 1
    elif value < -0.001:
        return -1
    else:
        return 0
 
def draw_traj(traject):
    dsmin = 0.05;
    x0 = traject[0]
    y0 = traject[1]
    theta0 = traject[2]
    r = traject[4]
    ds0 = traject[3]

    floor_ds  = math.fabs(ds0)/dsmin
    remian_ds = math.fabs(ds0) - floor_ds * dsmin
    points = floor_ds

    xs = []
    ys = []
    if math.fabs(r) < 1:
        i = 0
        while i < points:
            x = i * math.cos(theta0) * dsmin*sign(ds0) + x0
            y = i * math.sin(theta0) * dsmin*sign(ds0) + y0
            xs.append(x)
            ys.append(y)
            i += 1
    else:
        ftheta = -sign(r) * math.pi / 2 + theta0
        Xr = x0 - math.fabs(r) * math.cos(ftheta)
        Yr = y0 - math.fabs(r) * math.sin(ftheta)
        step_th = math.fabs(dsmin/r) * sign(ds0*r)
        i = 0
        while i < points:
            x = math.cos(i * step_th + ftheta) * math.fabs(r) + Xr
            y = math.sin(i * step_th + ftheta) * math.fabs(r) + Yr
            xs.append(x)
            ys.append(y)
            i += 1
    return xs,ys

def drawPos(filename, label):
    file = open(filename)
    xs = []
    ys = []
    nth = ''
    while True:
        text = file.readline()
        if not text:
            break
        pos = text.split(",")

        xs.append(float(pos[0]))
        ys.append(float(pos[1]))

    file.close()

    if len(xs) > 0:
       #plt.plot(xs, ys, color='y', linestyle='--', linewidth=1, label=title)
       plt.scatter(xs, ys, color='y', marker='+', s=5)
    

def drawPath(filename):
    file = open(filename)
    group = 0
    rxs = []
    rys = []
    cValue = ['r','g','b', 'c', 'm', 'k'] 
    while True:
        text = file.readline()
        if not text or len(text) < 3:
            break
        path = text.split(",")
        traject = [float(path[0]), float(path[1]), float(path[2]), float(path[3]), float(path[4])]
        xs, ys = draw_traj(traject)
        if int(path[5]) == group:
            rxs.extend(xs)
            rys.extend(ys)
        else:
            linelable = 'path ' + str(group)
            plt.plot(rxs, rys, color=cValue[group % len(cValue)], linestyle='-', linewidth=1, label=linelable)
            group = int(path[5])
            rxs = xs
            rys = ys
    file.close()
    
    if len(rxs) > 0:
        linelable = 'path ' + str(group)
        plt.plot(rxs, rys, color=cValue[group % len(cValue)], linestyle='-', linewidth=1, label=linelable)

def drawSlot(filename):
    file = open(filename)
    count = 0
    rxs = []
    rys = []
    cValue = ['r','g','b', 'c', 'm', 'k'] 
    while True:
        text = file.readline()
        if not text or len(text) < 3:
            break
        slots = text.split(",")
        xs = [float(slots[0]), float(slots[1]), float(slots[2]), float(slots[3]), float(slots[4]), float(slots[5])]

        text = file.readline()
        slots = text.split(",")
        ys = [float(slots[0]), float(slots[1]), float(slots[2]), float(slots[3]), float(slots[4]), float(slots[5])]
        slotlable = 'slot ' + str(count)
        plt.plot(xs, ys, color=cValue[count % len(cValue)], linestyle='-', linewidth=2, label=slotlable)
        count = count + 1

def drawObj(filename):
    file = open(filename)
    rxs = []
    rys = []
    cValue = ['r','g','b', 'c', 'm', 'k'] 
    while True:
        text = file.readline()
        if not text or len(text) < 3:
            break
        slots = text.split(",")
        xs = [float(slots[0]), float(slots[2])]
        ys = [float(slots[1]), float(slots[3])]
        count = int(slots[4])
        plt.plot(xs, ys, color=cValue[count % len(cValue)], linestyle='-', linewidth=2)

def drawAvm(filename):
    file = open(filename)
    count = 0
    rxs = []
    rys = []
    cValue = ['r','g','b', 'c', 'm', 'k'] 
    while True:
        text = file.readline()
        if not text or len(text) < 3:
            break
        slots = text.split(",")
        xs = [float(slots[0]), float(slots[2]), float(slots[4]), float(slots[6])]
        ys = [float(slots[1]), float(slots[3]), float(slots[5]), float(slots[7])]
        avmlable = 'avm'
        plt.plot(xs, ys, color=cValue[count % len(cValue)], linestyle='--', linewidth=2, label=avmlable)


print('...Waiting for message...')

fig = plt.figure()
plt.grid(linestyle='--')
plt.gca().set_aspect(1)
plt.tight_layout()

drawPath('fpath.txt')
drawPos('fpos.txt', 'boot points')
drawSlot('fslot.txt')
drawObj('fobj.txt')
drawAvm('favm.txt')
#drawPos('beizer.txt')
plt.legend()
plt.show()