#!/usr/bin/env python3
from bottle import *
import os
import sys
sys.path.append("/home/amigos/git")
import n2lite
import pandas
import csv
from datetime import datetime

num_history = 1000

db_path = "/home/amigos/data/db/ctrl_log.db"
db_path2 = "/home/amigos/data/db/observation_log.db"
aa = n2lite.N2lite(db_path)
bb = n2lite.N2lite(db_path2)

template_html = """
<html>
  <head><title>観測ログ</title></head>
  <meta http-equiv="refresh" content="10" >
  <link rel="stylesheet" type="text/css" href="/static/style.css"/>
  <body>
    <p>観測ログ</p>
    最終更新時間[UTC]：{last_update}<br>
    最終更新時間[JST]：{last_update_jst}
    <a href=/markdown_table2><p>[Observation log] wiki貼り付け用ページ</a>
    <a href=/markdown_table><p>[Controller log] wiki貼り付け用ページ</a>
    <a href=/><p>元のページ</p></a>
    <!-- <a href=/download/sample.txt><p> download test</a> 
    <a href=/download/sample.txt><p>wiki貼り付け用ページ</a>-->
    {table}
  </body>
</html>
"""
num_history = -1*num_history


@route('/static/<file_path:path>')
def static(file_path):
    return static_file(file_path, root='../contents')


def to_wiki(data_frame):
    data_frame.columns

def t(timestamp):
    a = datetime.utcfromtimestamp(timestamp)
    stra = a.strftime('%Y-%m-%d %H:%M:%S')
    return stra

@route("/")
def index():
    return template("index")

@route("/ctrl_log")
def index1():
    a =  aa.read_as_pandas("log2")
    df = pandas.DataFrame(a)
    times = list(df.time)
    xx = list(map(t,times))
    dff = pandas.DataFrame({"time":times, "times":xx})
    aaa = pandas.merge(dff,df, on="time")
    del aaa["time"]
    aaa = aaa[num_history:]
    now = datetime.utcnow()
    jst =now + timedelta(hours=9)
    with open('sample.html', 'w') as f:
        f.write(template_html.format(table=aaa.to_html(classes='mystyle'),last_update = now.strftime('%Y-%m-%d %H:%M:%S'),last_update_jst = jst.strftime(('%Y-%m-%d %H:%M:%S'))))
    return template("sample")

@route("/obs_log")
def index2():
    a =  bb.read_as_pandas("observation_log")
    df = pandas.DataFrame(a)
    a = a[num_history:]
    now = datetime.utcnow()
    jst =now + timedelta(hours=9)
    with open('sample2.html', 'w') as f:
        f.write(template_html.format(table=a.to_html(classes='mystyle'),last_update = now.strftime('%Y-%m-%d %H:%M:%S'),last_update_jst = jst.strftime(('%Y-%m-%d %H:%M:%S'))))
    return template("sample2")

@route("/markdown_table")
def return_wiki():
    data_frame = get_data("log2")
    return to_wiki(data_frame)

@route("/markdown_table2")
def return_wiki2():
    a =  bb.read_as_pandas("observation_log")
    df = pandas.DataFrame(a)
    return to_wiki2(df)

def get_data(table_name):
    a =  aa.read_as_pandas(table_name)
    df = pandas.DataFrame(a)
    times = list(df.time)
    xx = list(map(t,times))
    dff = pandas.DataFrame({"time":times, "times":xx})
    aaa = pandas.merge(dff,df, on="time")
    del aaa["time"]
    return aaa

def output_txt_file(d_frame):
    now = datetime.utcnow()
    with open("../download/sample.txt", "w") as f:
        csv_out = csv.writer(f)
        csv_out.writerows(d_frame.values)
        
@route('/download/<file_path:path>')
def download_static(file_path):
    return static_file(file_path, root='../contents', download=True)

def test():
    a = aa.read_as_pandas("log2")
    df = pandas.DataFrame(a)
    output_txt_file(df)
    
def to_wiki(df):
    temp = "|{}"
    tmp =""
    tmp2 = ""
    tmp3 = ""
    for i in list(df.columns):
        tmp += "|{}".format(i)
        tmp2 += "|---"
    tmp+= "|<br>"
    tmp2+="|<br>"
    tmp4 = "|{}"
    tmp3 = tmp4*len(list(df.iloc[0]))+"|<br>"
    tmp5 = ""
    num_his = num_history
    if len(df.index) < abs(num_history):
        num_his = -1*len(df.index)
    #for i in range(num_his):
    for i in range(num_his, 0):
        tmp5 += tmp3.format(df.iloc[i][0],df.iloc[i][1],df.iloc[i][2],df.iloc[i][3])
    return tmp+tmp2+tmp5 #+"<a href=/>戻る</a>"

def to_wiki2(df):
    temp = "|{}"
    tmp =""
    tmp2 = ""
    tmp3 = ""
    for i in list(df.columns):
        tmp += "|{}".format(i)
        tmp2 += "|---"
    tmp+= "|<br>"
    tmp2+="|<br>"
    tmp4 = "|{}"
    tmp3 = tmp4*len(list(df.iloc[0]))+"|<br>"
    tmp5 = ""
    num_his = num_history
    if len(df.index) < abs(num_history):
        num_his = -1*len(df.index)
    #for i in range(num_his):
    for i in range(num_his, 0):
        tmp5 += tmp3.format(df.iloc[i][0],df.iloc[i][1],df.iloc[i][2],df.iloc[i][3], df.iloc[i][4])
    return tmp+tmp2+tmp5 #+"<a href=/>戻る</a>"

run(host="0.0.0.0", port=8999, debug=True, reloader=True)
#test()
