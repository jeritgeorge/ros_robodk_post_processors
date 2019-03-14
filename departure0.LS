/PROG  departure0
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "test_comment";
PROG_SIZE	= 0;
CREATE		= DATE 31-12-14  TIME 12:00:00;
MODIFIED	= DATE 31-12-14  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 17;
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
   1:  GO[12:0]=1 ;
   2:  UTOOL_NUM=0 ;
   3:  ! UT0:0.0,0.0,0.0,0.0, ;
   4:  ! -0.0,0.0 ;
   5:  UFRAME_NUM=1 ;
   6:  ! UF1:0.0,0.0,0.0,0.0, ;
   7:  ! -0.0,0.0 ;
   8:  DO[96]=ON ;
   9:  DO[97]=OFF ;
  10:  WAIT DI[96]=ON ;
  11:  DO[96]=OFF ;
  12:J P[1] 10% FINE ;
  13:J P[2] 8% FINE ;
  14:  DO[97]=ON ;
  15:  DO[96]=OFF ;
  16:  WAIT DI[97]=ON ;
  17:  DO[97]=OFF ;
/POS
P[1]{
   GP1:
    UF : 1, UT : 0,    
	J1=    0.000 deg,	J2=    0.000 deg,	J3=    0.300 deg,
	J4=    0.600 deg,	J5=    0.500 deg,	J6=    10.000 deg,
	E1=  1000.000  mm,	E2=  1000.000  mm
};
P[2]{
   GP1:
    UF : 1, UT : 0,    
	J1=    0.000 deg,	J2=    0.000 deg,	J3=    0.000 deg,
	J4=    0.000 deg,	J5=    0.000 deg,	J6=    0.000 deg,
	E1=  1000.000  mm,	E2=  1000.000  mm
};
/END
