import sys

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
        raw.append(text.strip('\n'))
        vs.append(pos[6] + ',' + pos[7])
        bs.append(pos[16] + ',' + pos[17])
        es.append(pos[18] + ',' + pos[19])
        #print(vs,bs,es)
        #break
    return bs,es,vs,raw

def MergeAvmSlot(bs,es, raws):
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
                    temp = avmslot.get(key)
                    temp.append(raws[i])
                    flag = 1
                    break

            if flag == 0:
                avmslot[slotid] = [raw[i]]
                slottarget[slotid] = [targetx, targety]
                slotid = slotid + 1
    return avmslot

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
        filename = 'slot_' + str(key) + '.txt'
        savetoFile(value, filename)