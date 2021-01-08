import mysql.connector
import time
from datetime import datetime, timezone, date,timedelta
import pandas as pd


class BabyTrackerDB:
  _user=""
  _pass=""
  _host=""
  _port=3306
  _dbSchema=""
  _dbCNX=None

  def __init__(self, user, password, host, schema, port=3306):
    self._user=user
    self._pass=password
    self._host=host
    self._port=port
    self._dbSchema=schema

  def InsertNewEntry(self, idTracker, theDate, latitude, longitude, altitude, speed, numRetries=10):
    if not self.__EnsureDbConnected(numRetries):
      return False

    try:
      theDate=datetime.fromtimestamp(int(theDate)) #, tzinfo=timezone('Etc/GMT+1')
      mycursor = self._dbCNX.cursor()
      sql="INSERT INTO LocationHistory (IDTRACKER, LocDate, Latitude, Longitude, Altitude, Speed) VALUES (%s, %s, %s, %s, %s, %s)"
      values=(idTracker, theDate, latitude, longitude, altitude, speed)
      mycursor.execute(sql, values)
      self._dbCNX.commit()
    except Exception as err:
      print("Exception trying to insert into DB!: ", err)


  def __Connect(self):
    """Tries to connect to the mysql db. Returns true or false"""
    db_config = {
        'user': self._user,
        'password': self._pass,
        'host': self._host,
        'port': self._port,
        'database': self._dbSchema
    }
    try:
        self._dbCNX = mysql.connector.connect(**db_config)
    except mysql.connector.Error as err:
        if err.errno == mysql.connector.errorcode.ER_ACCESS_DENIED_ERROR:
            print("Something is wrong with your user name or password:", err)
        elif err.errno == mysql.connector.errorcode.ER_BAD_HOST_ERROR:
            print("Cannot connect to host:", err)
        elif err.errno == mysql.connector.errorcode.ER_BAD_DB_ERROR:
            print("Database does not exist:", err)
        else:
            print("Some Error occurred while connecting to DB:", err)

    return self._dbCNX and self._dbCNX.is_connected()


  def __EnsureDbConnected(self, numRetries=10):
    """Checks if the mysql connection is connected. 
    If its not, retries until a connection is made (for numRetries attempts...)"""
    while not self._dbCNX or not self._dbCNX.is_connected():
        if self._dbCNX:
            print("Connection to DB failed, retrying after 30s...")
            time.sleep(30)  # delay 30 seconds and try again
        if self.__Connect() :
          break
        numRetries-=1
        if numRetries<=0 :
          break

    return self._dbCNX and self._dbCNX.is_connected()

  def GetTracks(self,IDTracker,numLimit,fromData: datetime=None, numRetries=10):
    if(fromData is None): 
        fromData=date.today()
    fromData=fromData + timedelta(days=1)
    if not self.__EnsureDbConnected(numRetries):
        return False
    if (numLimit==0):
        numLimit=10
    sql="SELECT Latitude, Longitude, LocDate FROM LocationHistory WHERE IDTRACKER=%s AND LocDate<=%s ORDER BY IDTRACKER, LOCDATE DESC LIMIT %s"
    mycursor = self._dbCNX.cursor()
    df=pd.read_sql(sql, self._dbCNX, params=(IDTracker, fromData,numLimit))
    mycursor.close()
    return df