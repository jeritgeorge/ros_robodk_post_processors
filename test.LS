/PROG  test
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 0;
CREATE		= DATE 31-12-14  TIME 12:00:00;
MODIFIED	= DATE 31-12-14  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 19;
MEMORY_SIZE	= 0;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:  UTOOL_NUM=0 ;
   2:  ! UT0:0.0,0.0,0.0,0.0, ;
   3:  ! -0.0,0.0 ;
   4:  UFRAME_NUM=1 ;
   5:  ! UF1:0.0,0.0,0.0,0.0, ;
   6:  ! -0.0,0.0 ;
   7:J P[1] 10% FINE ;
   8:L P[2] 20mm/sec CNT2 ;
   9:L P[3] 20mm/sec CNT2 ;
  10:  MESSAGE[A run message] ;
  11:  WAIT  1.00(sec) ;
  12:  DO[1]=ON ;
  13:  GO[12:0]=1 ;
  14:  $WAITTMOUT=0 ;
  15:  WAIT DI[2]=ON TIMEOUT, LBL[1] ;
  16:  MESSAGE[Timed out for LBL 1] ;
  17:  PAUSE ;
  18:  LBL[1] ;
  19:  MY_FUNC;
/POS
P[1]{
   GP1:
    UF : 1, UT : 0,    
	J1=    0.000 deg,	J2=    0.000 deg,	J3=    0.000 deg,
	J4=    0.000 deg,	J5=    0.000 deg,	J6=    0.000 deg,
	E1=     0.000  mm,	E2=     0.000  mm
};
P[2]{
   GP1:
    UF : 1, UT : 0,        CONFIG : 'N U T, 0, 0, 0',
	X =  1000.000  mm,	Y =   500.000  mm,	Z =     0.000  mm,
	W =     0.000 deg,	P =    -0.000 deg,	R =     0.000 deg,
	E1=     0.000  mm,	E2=     0.000  mm
};
P[3]{
   GP1:
    UF : 1, UT : 0,    
	J1=    0.000 deg,	J2=    0.000 deg,	J3=    0.000 deg,
	J4=    0.000 deg,	J5=    0.000 deg,	J6=    0.000 deg,
	E1=     0.000  mm,	E2=     0.000  mm
};
/END
