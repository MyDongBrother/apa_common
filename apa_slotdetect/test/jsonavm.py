import sys
import json
 

def parsePath(infile):
    fin = open(infile)

    replancount = 0
    paths=[]
    while True:
        text = fin.readline()
        if not text:
            break

        result = json.loads(text)
        veh = result["curpos"]
        slotsdict = result["recv"]
        jsonstr = json.dumps(slotsdict)
        
        print(jsonstr)
        start = jsonstr.find('"id":')
        while start != -1:
            print(start)
            start = jsonstr.find('"id":', start + 4)
            
        slots = json.loads(jsonstr)
        print(slots['slot95'])
        
        
        break
    fin.close()
    
    
parsePath('2023_09_06_16_48_35_jsonavm.txt')    