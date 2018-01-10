"""                                                                                                                           
:: 1p85_server python client ::                                                                                               
                                                                                                                              
config.py                                                                                                                     
                                                                                                                              
[history]                                                                                                                     
2010/11/4 2010ver created nishimura                                                                                           
                                                                                                                              
"""

# ------ HOST CONFIG --------                                                                                                 

HOST_OPU_1p85pc1 = '157.16.88.54'
HOST_OPU_1p85pc2 = '157.16.88.58'
HOST_OPU_1p85dfs = '157.16.88.50'
HOST_NRO_1p85pc1 = '133.40.94.81'#'1p85pc1.nro.nao.ac.jp'                                                                     
HOST_NRO_1p85pc2 = '133.40.94.82'#'1p85pc2.nro.nao.ac.jp'                                                                     
HOST_NRO_1p85dfs = '133.40.94.85'#'1p85dfs.nro.nao.ac.jp'                                                                     

HOST_1p85pc1 = HOST_NRO_1p85pc1
HOST_1p85pc2 = HOST_NRO_1p85pc2
HOST_1p85dfs = HOST_NRO_1p85dfs


# ------ DATABASE CONFIG --------                                                                                             

DB_HOST   = HOST_1p85pc2
DB_PORT   = 3306
DB_USER   = '1p85_viewer'
DB_PASSWD = 'apples'
DB_ADMIN  = '1p85_server'
DB_ADPASS = 'ogawahideo'
DB_DB     = '1p85'


# ------ SERVER CONFIG --------                                                                                               

MOTOR_HOST   = HOST_1p85pc1
MOTOR_PORT   = 12122
MOTOR_TABLE  = 'motor'

LOAD_HOST    = HOST_1p85pc1
LOAD_PORT    = 12124
LOAD_TABLE   = 'load'

SERIAL_HOST  = HOST_1p85pc1
SERIAL_PORT  = 12128
SERIAL_TABLE = 'serial'

CONT_HOST    = HOST_1p85pc1
CONT_PORT    = 12123
CONT_TABLE   = 'cont'

IO_HOST      = HOST_1p85pc1
IO_PORT      = 12127
IO_TABLE     = 'io'

GPIB_HOST    = HOST_1p85pc1
GPIB_PORT    = 12125
GPIB_TABLE   = 'gpib'

CCD_HOST     = HOST_1p85pc1
CCD_PORT     = 12126
CCD_TABLE    = 'ccd'

DFS_HOST     = HOST_1p85dfs
DFS_PORT     = 52700
DFS_TABLE    = 'dfs'

