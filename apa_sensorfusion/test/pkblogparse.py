import sys

#1675679946:772609 flag:0 type:3 bfirst_rev: 0 vel:0.000000 g_mov_stat:000 curpos:4.405954,0.001882,0.001151,4.405957 radar:2550,2550,2550,2550,2630,3630,5100,5100,0000,0000,0000,0000 left:right:
def parsedata(infile):
    fin = open(infile)

    result=[]
    lastodm = -1.0
    while True:
        text = fin.readline()
        if not text:
            break

        key = ' flag:'
        start = text.find(key)
        stamp = text[0:start].strip()
        
        key = 'curpos:'
        start = text.find(key, start)
        if start == -1:
            continue

        start = start + len(key)
        key = ' '
        end = text.find(key, start)
        curpos = text[start:end].strip().split(',')
        #print(curpos)

        key = 'radar:'
        start = text.find(key)
        if start == -1:
            continue

        start = start + len(key)
        key = ' '
        end = text.find(key, start)
        radardata = text[start:end].strip().split(',')
        #print(radardata)
        
        curodm = float(curpos[3])
        if curodm - lastodm > 0.02:
            reccode =[stamp]
            reccode.extend(curpos)
            reccode.extend(radardata)
            strdata = ",".join(reccode)
            result.append(strdata)
            lastodm = curodm
    fin.close()
    return result

def saveToFile(outfile, result):
    fout = open(outfile, 'w')
    for item in result:
        fout.write(item + '\n')
    fout.close()

if __name__ == "__main__":
    count = len(sys.argv)
    logfile = 'flog.txt'
    if count == 2:
        logfile = sys.argv[1]
    
    result = parsedata(logfile)
    saveToFile('pkbradar.txt', result)