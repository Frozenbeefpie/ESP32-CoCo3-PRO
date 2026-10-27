import sys
raw=open('ramtest.raw','rb').read(); org=(raw[0]<<8)|raw[1]; data=raw[2:]
binf=bytes([0,len(data)>>8,len(data)&255,org>>8,org&255])+data+bytes([0xFF,0,0,org>>8,org&255])
bas=open('RAMTEST.BAS.txt').read().strip().replace('\r','').replace('\n','\r').encode('ascii')+b'\r'
dsk=bytearray(b'\xFF'*161280)
def off(tr,sec): return (tr*18+sec-1)*256
def gran_ts(g): tr=g//2; tr=tr+1 if tr>=17 else tr; return tr,(g%2)*9+1
fat=bytearray(b'\xFF'*256); fat[68:]=b'\x00'*(256-68)
nextg=[0]; dirents=[]
def store(name,ext,typ,asc,content):
    gs=[]; pos=0
    while pos<len(content) or not gs:
        g=nextg[0]; nextg[0]+=1; gs.append(g)
        tr,s=gran_ts(g); chunk=content[pos:pos+9*256]
        dsk[off(tr,s):off(tr,s)+len(chunk)]=chunk; pos+=9*256
    for i,g in enumerate(gs[:-1]): fat[g]=gs[i+1]
    last=len(content)-(len(gs)-1)*9*256; secs=(last+255)//256; lastbytes=last-(secs-1)*256
    fat[gs[-1]]=0xC0|secs
    e=bytearray(32); e[0:8]=name.ljust(8).encode(); e[8:11]=ext.encode(); e[11]=typ; e[12]=asc; e[13]=gs[0]
    e[14]=lastbytes>>8; e[15]=lastbytes&255
    dirents.append(e)
store('RAMTEST','BAS',0,0xFF,bas)
store('RAMTEST','BIN',2,0,binf)
dsk[off(17,2):off(17,2)+256]=fat
d=bytearray(b'\xFF'*256)
for i,e in enumerate(dirents): d[i*32:i*32+32]=e
d[len(dirents)*32]=0x00  # (0 = entrée effacée ; 0xFF = fin du répertoire) -> laisser FF
d[len(dirents)*32]=0xFF
dsk[off(17,3):off(17,3)+256]=d
open('RAMTEST.DSK','wb').write(dsk); print('ok', len(dsk), 'granules', nextg[0])
