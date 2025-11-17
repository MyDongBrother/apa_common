import sys
import math
import json

def ParseAvmPos(filename):
    file = open(filename)
    raw = []
    vs = []
    bs = []
    es = []

    while True:
        text = file.readline()
        if not text:
            break

        pos = text.strip('\n').split(',')
        if len(pos) < 10:
            print('error line: ', text)
            break
        #print(pos)
        #raw.append(text.strip('\n'))
        vs.append(pos[7] + ',' + pos[8])
        bs.append(pos[17] + ',' + pos[18])
        es.append(pos[19] + ',' + pos[20])
        out = pos[2] + ',' + pos[7] + ',' + pos[8] + ' '
        out = out + '"stamp\":' + pos[1] + ','
        out = out + '\"pos\":[' + pos[3] + ',' + pos[4] + ',' + pos[5] + '],'
        out = out + '\"be\":[' + pos[12] + ',' + pos[13] + ',' + pos[14] + ',' + pos[15]  + '],'
        out = out + '\"cd\":[' + pos[22] + ',' + pos[23] + ',' + pos[24] + ',' + pos[25]  + '],'
        out = out + '\"stop\":[' + pos[32] + ',' + pos[33] + ',' + pos[34] + ',' + pos[35] + ']'
        raw.append(out)
        #print(vs,bs,es)
        #print(out)
        #break
    return bs,es,vs,raw

def MergeAvmSlot(bs,es,raws):
    count = len(bs)
    slotid = 1
    slottarget = dict()
    avmslot = dict()

    for i in range(count):
        b = bs[i]
        e = es[i]
        
        bx = float(b.split(',')[0])
        by = float(b.split(',')[1])

        ex = float(e.split(',')[0])
        ey = float(e.split(',')[1])

        targetx = (bx + ex) * 0.5
        targety = (by + ey) * 0.5

        if len(slottarget) == 0:
            avmslot[slotid] = [raws[i]]
            slottarget[slotid] = [targetx, targety]
            slotid = slotid + 1
        else:
            flag = 0
            for key,value in slottarget.items():
                dist = (targetx - value[0]) * (targetx - value[0]) + (targety - value[1]) * (targety - value[1])
                if (dist < 1.5):
                    #if key == 1:
                    #    print(b + ' ' + e + ' ' + str(value[0]) + ' ' + str(value[1]) + '\n')
                    temp = avmslot.get(key)
                    temp.append(raws[i])
                    flag = 1
                    break

            if flag == 0:
                avmslot[slotid] = [raw[i]]
                slottarget[slotid] = [targetx, targety]
                slotid = slotid + 1
    return avmslot

def CalcSpeed(datas):
    lasttime = -1;
    lastpos = [0,0]
    speed = 0.0;
    result = []
    for item in datas:
        head = item.split(' ')[0]
        data = item.split(' ')[1]
        
        pos = head.split(',')
        if lasttime < 0:
            lasttime = int(pos[0])
            lastpos = [float(pos[1]), float(pos[2])]
            continue
        else:
            curtime = int(pos[0])
            curpos = [float(pos[1]), float(pos[2])]
            dist = math.sqrt((curpos[0] - lastpos[0]) * (curpos[0] - lastpos[0]) + (curpos[1] - lastpos[1]) * (curpos[1] - lastpos[1]))
            speed = dist * 1000.0 / (curtime - lasttime)
            #print(lasttime, curtime, lastpos,curpos,speed)
            lasttime = curtime
            lastpos[0] = curpos[0]
            lastpos[1] = curpos[1]
            result.append('{\"speed\":%06f'%speed + ',' + data + '}')
    return result

def savetoFile(datalist, filename):
    fo = open(filename, 'w')
    for datastr in datalist:
        fo.write(datastr + '\n')
    fo.close()

if __name__ == "__main__" :
    if len(sys.argv) != 2:
        print('must input paramaters\n')
        exit()

    bs, es, vs, raw = ParseAvmPos(sys.argv[1])
    slots = MergeAvmSlot(bs, es, raw)
    for key, value in slots.items():
        #filename = 'slot_' + str(key) + '.txt'
        filename = 'slot_' + str(key) + '_json.txt'
        result = CalcSpeed(value)
        savetoFile(result, filename)
