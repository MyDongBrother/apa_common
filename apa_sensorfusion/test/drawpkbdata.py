import sys
import math
import matplotlib.pyplot as plt

def sign(value):
    if value > 0.001:
        return 1
    elif value < -0.001:
        return -1
    else:
        return 0

def ObjectLocation(paras, radar, vel):
    sintmp1 = math.sin(paras[0])
    costmp1 = math.cos(paras[0])

    sintmp2 = math.sin(vel[2])
    costmp2 = math.cos(vel[2])

    xDetect = paras[1] + (radar * costmp1 * 0.001)
    yDetect = paras[2] + (radar * sintmp1 * 0.001);

    x = xDetect * costmp2 - yDetect * sintmp2 + vel[0]
    y = xDetect * sintmp2 + yDetect * costmp2 + vel[1]
    return x,y

def NameToIdx(name):
    namelist = ['ROL', 'RCL', 'RCR', 'ROR', 'RSL', 'RSR', 'FSL', 'FSR']
    return namelist.index(name)

def RadarLocation(idx):
    loclist = [
        [-0.9100,0.7355,2.4435], #ROL
        [-1.0300,0.2975,3.1416], #'RCL':
        [-1.0300,-0.2975,-3.1416], #'RCR':
        [-0.9100,-0.7355,-2.4435], #'ROR':
        [-0.4300,0.9575,1.6057],   #'RSL':
        [-0.4300,-0.9575,-1.6057], #'RSR':
        [3.3300,0.9575,1.5359],    #'FSL':
        [3.3300,-0.9575,-1.5359]  #'FSR':
      ]
    return loclist[idx]

def drawdata(infile, radarname):
    fin = open(infile)

    idx = NameToIdx(radarname)
    radarloc = RadarLocation(idx)

    xs = []
    ys = []
    vxs = []
    vys = []

    while True:
        text = fin.readline()
        if not text:
            break
        
        datas = text.strip().split(',')
        stamp = datas[0]
        curpos = [float(datas[1]), float(datas[2]), float(datas[3])]
        distval = int(datas[5 + idx])

        if distval == 2550 or distval == 5100:
            continue

        x,y = ObjectLocation([radarloc[2],radarloc[0],radarloc[1]], distval, curpos)
        print(stamp, curpos, x, y)
        xs.append(x)
        ys.append(y)
        vxs.append(curpos[0])
        vys.append(curpos[1])

    fin.close()
    if len(xs) > 0:
        #plt.plot(xs, ys, color='y', linestyle='--', linewidth=1, label=title)
        plt.scatter(xs, ys, color='r', marker='+', s=5, label=radarname)    
        plt.scatter(vxs, vys, color='g', marker='+', s=5)    

def drawPos(filename):
    file = open(filename)
    xs = []
    ys = []
    nth = ''
    while True:
        text = file.readline()
        if not text:
            break
        datas = text.strip().split(',')

        xs.append(float(datas[1]))
        ys.append(float(datas[2]))

    file.close()

    if len(xs) > 0:
       #plt.plot(xs, ys, color='y', linestyle='--', linewidth=1, label=title)
       plt.scatter(xs, ys, color='y', marker='+', s=1)

print('...Waiting for message...')

fig = plt.figure()
plt.grid(linestyle='--')
plt.gca().set_aspect(1)
plt.tight_layout()

count = len(sys.argv)
radarname = ''
if count == 2:
    radarname = sys.argv[1]
else:
    print('valid input: ROL/RCL/RCR/ROR/RSL/RSR/FSL/FSR')
    exit()

drawPos('pkbradar.txt')    
drawdata('pkbradar.txt', radarname)

plt.legend()
plt.show()