/PROG  fromHome
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "test_comment";
PROG_SIZE	= 0;
CREATE		= DATE 31-12-14  TIME 12:00:00;
MODIFIED	= DATE 31-12-14  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 8;
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
   1:  UTOOL_NUM=1 ;
   2:  ! UT1:0.0,0.0,0.0,0.0, ;
   3:  ! -0.0,0.0 ;
   4:  UFRAME_NUM=0 ;
   5:  ! UF0:0.0,0.0,0.0,0.0, ;
   6:  ! -0.0,0.0 ;
   7:J P[1] 10% FINE ;
   8:J P[2] 8% FINE ;
/POS
P[1]{
   GP1:
    UF : 0, UT : 1,    
	J1=    -90.000 deg,	J2=    -65.000 deg,	J3=    0.000 deg,
	J4=    -180.000 deg,	J5=    -60.000 deg,	J6=    10.000 deg,
	E1= -1000.000  mm,	E2= -1000.000  mm
};
P[2]{
   GP1:
    UF : 0, UT : 1,    
	J1=    -90.000 deg,	J2=    -65.000 deg,	J3=    0.000 deg,
	J4=    180.000 deg,	J5=    -60.000 deg,	J6=    0.000 deg,
	E1= -1500.000  mm,	E2= -1500.000  mm
};
/END
