
""" -- comment --                                                                                                             
:: 1p85_server python client ::                                                                                               
                                                                                                                              
client.py                                                                                                                     
                                                                                                                              
[history]                                                                                                                     
2010/11/02 2010ver created nishimura                                                                                          
                                                                                                                              
"""

# ------ settings ------                                                                                                      

REPLY_END = 'SERVER_READY'

HELP_STR_SOCKET = """                                                                                                         
[ --- SOCKET USAGE --- ]                                                                                                      
 open(host,port)                : start socket connection.                                                                    
 close()                        : end socket connection.                                                                      
 cancel()                       : send a cancel call.                                                                         
 _send_command(command,params)  : send a command and parameters.                                                              
"""

HELP_STR_DB = """                                                                                                             
[ --- DATABASE USAGE --- ]                                                                                                    
 db_print_log(last,start,end)      : print log data.                                                                          
 db_print_monitor(last,start,end)  : print status data.                                                                       
"""

HELP_STR_HELP = """                                                                                                           
[ --- HELP USAGE ---]                                                                                                         
 help()                      : print usage                                                                                    
 print_client_config()       : print client configures                                                                        
"""

# ------ start source code ------                                                                                             

#import socket, MySQLdb, numpy                                                                                                
import socket, numpy
#import config
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
import dfs_table

DEFAULT_DB_HOST = dfs_table.DB_HOST
DEFAULT_DB_PORT = dfs_table.DB_PORT
DB_USER   = dfs_table.DB_USER
DB_PASSWD = dfs_table.DB_PASSWD
DB_DB     = dfs_table.DB_DB


class Client(object):
    _socket = ''
    _db = ''

    _socket_host = ''
    _socket_port = ''
    _db_host = ''
    _db_port = ''
    _db_table_log = ''
    _db_table_monitor = ''

    print_socket = True

    def __init__(self, print_socket=True):
        self.print_socket = print_socket
        return

    def _init_socket(self, host, port):
        self._socket_host = host
        self._socket_port = port
        return

    def _init_db(self, host=DEFAULT_DB_HOST, port=DEFAULT_DB_PORT, table_log='', table_monitor=''):
        self._db_host = host
        self._db_port = port
        self._db_table_log = table_log
        self._db_table_monitor = table_monitor
        return

    def print_client_config(self):
        if self._socket=='': socket = '(unconnected)'
        else: socket = '(connection ok.)'
        if self._db=='': db = '(unconnected)'
        else: db = '(connection ok.)'
        print('[SOCKET]')
        print('host:port -> %s:%d   %s' % (self._socket_host, self._socket_port, socket))
        print('')
        print('[DATABASE]')
        print('host:port -> %s:%d   %s' % (self._db_host, self._db_port, db))
        print('log -> %s, monitor -> %s' % (self._db_table_log, self._db_table_monitor))
        return

    def help(self):
        self._print_common_help()
        print('sorry, no help document are available...')
        return

    def _print_common_help(self):
        print(HELP_STR_SOCKET)
        print(HELP_STR_DB)
        print(HELP_STR_HELP)
        return

    def open(self, host='', port=''):
        if host=='': host = self._socket_host
        if port=='': port = self._socket_port
        self._socket = ClientSocketIO(self)
        self._socket.connect(host, port)
        self._read_reply_loop()
        return

    def _send_command(self, command='', params=''):
        self._socket.write(command)
        self._socket.write(params)
        return

    def _set_reply_handler(self, __reply_handler_func):
        self.__reply_handler = __reply_handler_func
        return

    def __reply_handler(self, reply):
        print(reply)
        return

    def _read_reply_loop(self):
        while True:
            reply = self._socket.read_reply_line()
            if reply==REPLY_END: break
            self.__reply_handler(reply)
            continue
        return

    def close(self):
        self._socket.close()
        self._socket = ''
        return

    def cancel(self):
        self._send_command('OPERATION_CANCEL')
        return

    def __db_check(self):
        if self._db=='': self.__db_connect()
        return

    def __db_connect(self):
        #self._db = MySQLClient()
        return

    def db_print_log(self, last=10, start='', end=''):
        self.__db_check()
        self._db.print_table(self._db_table_log, last, start, end)
        return

    def db_print_monitor(self, last=10, start='', end=''):
        self.__db_check()
        self._db.print_table(self._db_table_monitor, last, start, end)
        return



#class ClientSocketIO():
class ClientSocketIO:
    def __init__(self, parent):
        self.parent = parent
        self.socket = ''
        self._in = ''
        self._out = ''
        return

    def connect(self, _ip, _port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((_ip, _port))
        self._in = self.socket.makefile('rb')
        self._out = self.socket.makefile('wb')
        if self.parent.print_socket==True: print('(socket) connection succeed. %s:%d'%(_ip, _port))
        return

    def close(self):
        self._in.close()
        self._out.close()
        self.socket.close()
        print('(socket) connection closed.')
        return
    
    def write(self, msg):
        self._out.write(msg.strip()+'\n')
        self._out.flush()
        return
    
    def read_reply_line(self):
        return self._in.readline().strip()
    
    def read_reply_bin(self, length):
        return self._in.read(length)



#class MySQLClient():
class MySQLClient:
    def __init__(self, host=DEFAULT_DB_HOST, port=DEFAULT_DB_PORT, user=DB_USER, passwd=DB_PASSWD, db=DB_DB):
        self.host = host
        self.port = port
        self.user = user
        self.passwd = passwd
        self.use_db = db
        self.db = MySQLdb.connect(host=self.host, port=self.port, user=self.user, passwd=self.passwd, db=self.use_db)
        self.cursor = self.db.cursor()
        return

    def execute(self, sql='', args=()):
        self.cursor.execute(sql, args)
        return self.cursor.fetchall()

    def get(self, table='', last=30, start='', end=''):
        """Get DB items. table="""
        if   table=='': return
        if start=='' and end=='': return self.__db_get_last(table, last)
        else: return self.__db_get_span(table, start, end)
        return

    def __db_get_span(self, table='', start='', end=''):
        sql = 'SELECT * FROM %s WHERE Time>"%s" AND Time<"%s"'%(table, start, end)
        return self.execute(sql)

    def __db_get_last(self, table='', last=100):
        sql = 'SELECT * FROM %s ORDER BY id DESC LIMIT %d'%(table, last)
        return self.execute(sql)

    def print_table(self, table='', last=10, start='', end=''):
        if table=='':
            print('no table selected.')
            return

        columns = numpy.array(self.execute('SHOW columns FROM %s'%(table)))
        items = numpy.array(self.get(table, last, start, end))
        rows = []
        rows.append(columns[:,0])
        for i in items:
            rows.append(i)
            continue
        rows = numpy.array(rows)
        self.rows = rows
        row_len = range(len(rows[0]))
        for j in range(len(rows[0])):
            item_len = []
            for i in range(len(rows)):
                rows[i][j] = '%s'%(rows[i][j])
                item_len.append(len(rows[i][j]))
                continue
            row_len[j] = max(item_len)
            continue
        
        def hori_line(row_len):
            line = '+'
            for i in row_len:
                line += '-'*(i+2) + '+'
                continue
            return line+'\n'

        lines = hori_line(row_len)
        count = 0
        for row in rows:
            lines += '|'
            item_num = 0
            for item in row:
                format = ' %%-%ds '%(row_len[item_num])
                lines += format%(item) + '|'
                item_num += 1
                continue
            lines += '\n'
            if count==0: lines += hori_line(row_len)
            count += 1
            continue
        lines += hori_line(row_len)
        print(lines)
        return

    
