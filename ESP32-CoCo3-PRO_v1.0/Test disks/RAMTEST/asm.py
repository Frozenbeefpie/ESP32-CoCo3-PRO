# Mini-assembleur 6809 (sous-ensemble) pour le test RAM/MMU. Deux passes.
import sys
EQU = dict(N=0x7F00, ERR=0x7F02, FIRST=0x7F04, BLK=0x7F06, BK8=0x7F08, BKX=0x7F09,
           FERR=0x7F0A, PASS=0x7F0C, MRES=0x7F0D, SAVB5=0x7F0E, SAVB6=0x7F0F, SAVFE=0x7F10,
           SAV1=0x7F18, TABLE=0x6010)
INH = {'COMB':[0x53],'LSLA':[0x48],'CLRA':[0x4F],'CLRB':[0x5F],'RTS':[0x39],'NOP':[0x12]}
IMM8 = {'ORCC':[0x1A],'ANDCC':[0x1C],'LDA':[0x86],'LDB':[0xC6],'EORA':[0x88],'EORB':[0xC8],
        'ANDA':[0x84],'ANDB':[0xC4],'CMPA':[0x81],'CMPB':[0xC1],'ORA':[0x8A]}
IMM16 = {'LDD':[0xCC],'ADDD':[0xC3],'SUBD':[0x83],'CMPD':[0x10,0x83],'LDX':[0x8E],'CMPX':[0x8C],
         'LDY':[0x10,0x8E],'CMPY':[0x10,0x8C]}
EXT = {'LDD':[0xFC],'STD':[0xFD],'CMPD':[0x10,0xB3],'LDA':[0xB6],'STA':[0xB7],'LDB':[0xF6],'STB':[0xF7],
       'EORB':[0xF8],'CMPA':[0xB1],'CMPB':[0xF1],'CLR':[0x7F],'COM':[0x73],'INC':[0x7C],'JSR':[0xBD]}
IDX = {'LDA':0xA6,'STA':0xA7,'STB':0xE7,'CMPB':0xE1,'EORB':0xE8}
POST = {',X+':0x80,',-X':0x82,',Y+':0xA0,',S+':0xE0}
BR8 = {'BNE':0x26,'BEQ':0x27,'BRA':0x20}
REG = {'CC':1,'A':2,'B':4,'DP':8,'X':0x10,'Y':0x20,'U':0x40,'PC':0x80,'D':6}

def val(s, labels):
    s=s.strip()
    if s.startswith('$'): return int(s[1:],16)
    if s in EQU: return EQU[s]
    if s in labels: return labels[s]
    if s.isdigit(): return int(s)
    return None

def assemble(src):
    labels={}
    for p in (1,2):
        out={}; pc=0
        for line in src.split('\n'):
            line=line.split(';')[0].rstrip()
            if not line.strip(): continue
            lab=None
            if not line[0].isspace():
                lab,_,line=line.partition(' ') if ' ' in line else (line,'','')
                labels[lab]=pc
            parts=line.split(None,1)
            if not parts: continue
            op=parts[0].upper(); arg=parts[1].strip() if len(parts)>1 else ''
            b=[]
            if op=='ORG': pc=val(arg,labels); 
            elif op=='FCB': b=[val(x,labels) for x in arg.split(',')]
            elif op in INH and not arg: b=INH[op]
            elif op in ('PSHS','PULS'):
                m=0
                for r in arg.split(','): m|=REG[r.strip()]
                b=[0x34 if op=='PSHS' else 0x35, m]
            elif op=='TFR': b=[0x1F,0x10]  # TFR X,D seulement
            elif op in BR8:
                t=val(arg,labels); off=(t-(pc+2)) if t is not None else 0
                if p==2 and not -128<=off<=127: sys.exit('branche hors portee: %s %s'%(op,arg))
                b=[BR8[op],off&0xFF]
            elif arg.startswith('#'):
                v=val(arg[1:],labels) or 0
                if op in IMM8: b=IMM8[op]+[v&0xFF]
                else: b=IMM16[op]+[(v>>8)&0xFF,v&0xFF]
            elif arg in POST: b=[IDX[op],POST[arg]]
            else:
                v=val(arg,labels); v=0 if v is None else v
                b=EXT[op]+[(v>>8)&0xFF,v&0xFF]
            if p==2 and op!='ORG':
                for i,x in enumerate(b): out[pc+i]=x
            if op!='ORG': pc+=len(b)
    return out,labels

SRC = open(sys.argv[1]).read()
mem,labels = assemble(SRC)
lo=min(mem); hi=max(mem)
img=bytes(mem.get(a,0x12) for a in range(lo,hi+1))
open(sys.argv[2],'wb').write(bytes([lo>>8,lo&0xFF])+img)
for k in ('SIG','FULL','MISC'): print(k, hex(labels[k]))
print('taille', len(img), 'de', hex(lo), 'a', hex(hi))
