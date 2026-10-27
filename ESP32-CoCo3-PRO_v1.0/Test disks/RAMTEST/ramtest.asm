; RAMTEST : test de la MMU et de la RAM du GIME (512 Ko / 2 Mo / 4 Mo)
        ORG $7000
SIG     ORCC #$50
        PSHS U,Y,X,D
        LDD #0
        STD ERR
        LDD #$FFFF
        STD FIRST
        LDX #TABLE
        LDD #0
S1      STD BLK
        JSR MAP
        LDA $4000
        STA ,X+
        LDA $4001
        STA ,X+
        LDA $5FFE
        STA ,X+
        LDA $5FFF
        STA ,X+
        LDD BLK
        STB $4000
        EORA #$A5
        STA $4001
        COMB
        STB $5FFE
        LDA #$5A
        STA $5FFF
        LDD BLK
        ADDD #1
        CMPD N
        BNE S1
        LDD #0
S2      STD BLK
        JSR MAP
        LDD BLK
        CMPB $4000
        BNE BAD
        EORA #$A5
        CMPA $4001
        BNE BAD
        COMB
        CMPB $5FFE
        BNE BAD
        LDA #$5A
        CMPA $5FFF
        BNE BAD
S2N     LDD BLK
        ADDD #1
        CMPD N
        BNE S2
        BRA S3
BAD     LDD ERR
        ADDD #1
        STD ERR
        LDD FIRST
        CMPD #$FFFF
        BNE S2N
        LDD BLK
        STD FIRST
        BRA S2N
S3      LDD N
S3L     SUBD #1
        STD BLK
        JSR MAP
        LDA ,-X
        STA $5FFF
        LDA ,-X
        STA $5FFE
        LDA ,-X
        STA $4001
        LDA ,-X
        STA $4000
        LDD BLK
        BNE S3L
        CLR $FF9B
        LDA #$3A
        STA $FFA2
        PULS D,X,Y,U
        ANDCC #$AF
        RTS
MAP     LDD BLK
        ANDA #3
        LSLA
        LSLA
        LSLA
        LSLA
        STA $FF9B
        STB $FFA2
        RTS

        ORG $7100
FULL    ORCC #$50
        PSHS U,Y,X,D
        LDA BKX
        ANDA #3
        LSLA
        LSLA
        LSLA
        LSLA
        STA $FF9B
        LDB BK8
        STB $FFA2
        LDD #0
        STD FERR
        CLR PASS
FP      LDX #$4000
FW      TFR X,D
        PSHS A
        EORB ,S+
        EORB BK8
        EORB PASS
        STB ,X+
        CMPX #$6000
        BNE FW
        LDX #$4000
FV      TFR X,D
        PSHS A
        EORB ,S+
        EORB BK8
        EORB PASS
        CMPB ,X+
        BEQ FOK
        LDD FERR
        ADDD #1
        STD FERR
FOK     CMPX #$6000
        BNE FV
        COM PASS
        BNE FP
        CLR $FF9B
        LDA #$3A
        STA $FFA2
        PULS D,X,Y,U
        ANDCC #$AF
        RTS

        ORG $7180
MISC    ORCC #$50
        PSHS U,Y,X,D
        CLR MRES
        LDX #$FFA8
        LDY #SAV1
M0      LDA ,X+
        ANDA #$3F
        STA ,Y+
        CMPX #$FFB0
        BNE M0
        LDA #$05
        STA $FFA2
        LDB $4000
        STB SAVB5
        LDA #$C3
        STA $4000
        LDA #$06
        STA $FFA2
        LDB $4000
        STB SAVB6
        LDA #$3C
        STA $4000
        LDA #$3A
        STA $FFA2
        LDX #T1MAP
        LDY #$FFA8
M1      LDA ,X+
        STA ,Y+
        CMPY #$FFB0
        BNE M1
        LDA #$01
        STA $FF91
        LDB $4000
        CLR $FF91
        CMPB #$3C
        BNE M2
        INC MRES
M2      LDA $FE80
        STA SAVFE
        EORA #$A5
        STA $FE80
        LDB #$3F
        STB $FFA2
        LDB $5E80
        LDA #$3A
        STA $FFA2
        LDA SAVFE
        EORA #$A5
        PSHS A
        CMPB ,S+
        BNE M3
        LDA MRES
        ORA #2
        STA MRES
M3      LDA SAVFE
        STA $FE80
        LDA #$05
        STA $FFA2
        LDA SAVB5
        STA $4000
        LDA #$06
        STA $FFA2
        LDA SAVB6
        STA $4000
        LDA #$3A
        STA $FFA2
        LDX #SAV1
        LDY #$FFA8
M4      LDA ,X+
        STA ,Y+
        CMPY #$FFB0
        BNE M4
        PULS D,X,Y,U
        ANDCC #$AF
        RTS
T1MAP   FCB $38,$39,$06,$3B,$3C,$3D,$3E,$3F
