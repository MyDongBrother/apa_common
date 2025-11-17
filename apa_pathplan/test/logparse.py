import sys

def parsePath(infile):
    fin = open(infile)

    replancount = 0
    paths=[]
    while True:
        text = fin.readline()
        if not text:
            break
        
        key = 'pathstr:'
        start = text.find(key)
        if start == -1:
            continue
            
        pathnum = 0
        data = text[start + len(key):].strip().split(',')
        pathnum = int(len(data) / 8)
        #print(data)

        if pathnum == 0:
            continue

        for index in range(pathnum):
            path = ''
            path = path + data[8 * index + 0] + ','
            path = path + data[8 * index + 1] + ','
            path = path + data[8 * index + 2] + ','
            path = path + data[8 * index + 3] + ','
            path = path + data[8 * index + 4] + ','
            path = path + str(replancount) + ','
            path = path + str(index) + '\n'
            paths.append(path)
        replancount = replancount + 1
    fin.close()
    return paths

def parseSlot(infile):
    fin = open(infile)

    replancount = 0
    objs = []
    linenumber = 0
    while True:
        text = fin.readline()
        if not text:
            break
        linenumber = linenumber + 1
        key = 'slotobj:'
        start = text.find(key)
        if start == -1:
            continue

        datas = text[start + len(key):].strip(' ').strip('\n').split(',')
        print(datas)
        print(len(datas))
        if len(datas) <12:
            print('line number: %d'%linenumber + ' %s'%text)
            continue
        xs = ''
        xs = xs + datas[0] + ','
        xs = xs + datas[2] + ','
        xs = xs + datas[4] + ','
        xs = xs + datas[6] + ','
        xs = xs + datas[8] + ','
        xs = xs + datas[10] + ','
        xs = xs + str(replancount) + '\n'

        ys = ''
        ys = ys + datas[1] + ','
        ys = ys + datas[3] + ','
        ys = ys + datas[5] + ','
        ys = ys + datas[7] + ','
        ys = ys + datas[9] + ','
        ys = ys + datas[11] + ','
        ys = ys + str(replancount) + '\n'
        objs.append(xs)
        objs.append(ys)

        replancount = replancount + 1
    fin.close()
    return objs

def parseSlotAvm(infile):
    fin = open(infile)

    replancount = 0
    objs = []
    linenumber = 0
    while True:
        text = fin.readline()
        if not text:
            break
        linenumber = linenumber + 1
        key = 'slotobj:'
        start = text.find(key)
        if start == -1:
            continue

        datas = text[start + len(key):].strip(' ').strip('\n').split(',')
        if len(datas) < 23:
            print('line number: %d'%linenumber + ' %s'%text)
            continue
        xs = ''
        xs = xs + datas[15] + ','
        xs = xs + datas[16] + ','
        xs = xs + datas[17] + ','
        xs = xs + datas[18] + ','
        xs = xs + datas[19] + ','
        xs = xs + datas[20] + ','
        xs = xs + datas[21] + ','
        xs = xs + datas[22] + ','
        xs = xs + str(replancount) + '\n'
        objs.append(xs)
        replancount = replancount + 1
    fin.close()
    return objs

def parsePos(infile):
    fin = open(infile)
    result = []
    while True:
        text = fin.readline()
        if not text:
            break
        pos = text.split(" ")

        if len(text) < 200 or len(pos) < 3:
            continue

        try:
            x=''
            y=''
            t=''
            nth=''
            for item in pos:
                start = item.find('cur_tra_nth:')
                if start != -1:
                    nth = item[len('cur_tra_nth:'):]

                start = item.find('curpos[0]:')
                if start != -1:
                    x = item[len('curpos[0]:'):];

                start = item.find('curpos[1]:')
                if start != -1:
                    y = item[len('curpos[1]:'):];

                start = item.find('curpos[2]:')
                if start != -1:
                    t = item[len('curpos[2]:'):];

                if x != "" and y != "" and t !='' and nth != "":
                    result.append(x +',' + y + ',' + t + ',' + nth + ',' + pos[0] + '\n')
                    break
        except:
            pass

    fin.close()
    return result

def parseObj(infile):
    fin = open(infile)
    result = []
    count = 0
    while True:
        text = fin.readline()
        if not text:
            break

        key = 'object:'
        start = text.find(key)
        if start == -1:
            continue

        datas = text[start + len(key):].strip(' ').strip('\n').split(',')
        print(datas)
        print(len(datas))
        counts = int(len(datas) / 4)
        for i in range(counts):
            objs = datas[i * 4] + ',' + datas[i * 4 + 1] + ',' + datas[i * 4 + 2] + ',' + datas[i * 4 + 3] + ',' + str(count) + ',' + str(i) + '\n'
            result.append(objs)
        count = count + 1
    fin.close()
    return result

def parseSpeed(infile):
    fin = open(infile)
    result = []
    while True:
        text = fin.readline()
        if not text:
            break

        key = 'cur_vel:'
        pos = text.find(key)
        if pos == -1:
            continue

        start = pos + len(key)
        pos = text.find(' ', start)
        if pos == -1:
            continue
        end = pos
        obj = text[start:end]
        if len(obj) > 0:
            result.append(obj + '\n')
    fin.close()
    return result

def saveToFile(outfile, result):
    fout = open(outfile, 'w')
    for item in result:
        fout.write(item)
    fout.close()


if __name__ == "__main__":
    count = len(sys.argv)
    logfile = 'flog.txt'
    if count == 2:
        logfile = sys.argv[1]
    
    result = parsePath(logfile)
    saveToFile('fpath.txt', result)
    
    result = parsePos(logfile)
    saveToFile('fpos.txt', result)
    
    result = parseSlot(logfile)
    saveToFile('fslot.txt', result)
    
    result = parseObj(logfile)
    saveToFile('fobj.txt', result)
    
    result = parseSpeed(logfile)
    saveToFile('fspd.txt', result)

    result = parseSlotAvm(logfile)
    saveToFile('favm.txt', result)
