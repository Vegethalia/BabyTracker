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

  def InsertNewEntry(self, idTracker, theDate, latitude, longitude, altitude, speed, battery=0.0, distance=0.0, numRetries=10):
    if not self.TryConnect(numRetries):
      return False

    try:
#      theDate=datetime.fromtimestamp(int(theDate)) #, tzinfo=timezone('Etc/GMT+1')
      mycursor = self._dbCNX.cursor()
      sql="INSERT INTO LocationHistory (IDTRACKER, LocDate, Latitude, Longitude, Altitude, Speed, Battery, Distance) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
      values=(idTracker, theDate, latitude, longitude, altitude, speed, battery, distance)
      mycursor.execute(sql, values)
      self._dbCNX.commit()
    except Exception as err:
      print("Exception trying to insert into DB!: ", err)


  def InsertEntries(self, missingRows):
    """Insert the list of entries into the DB"""
    if(not self.TryConnect(1)):  
      return None;

    try:
      for row in missingRows:
        self.InsertNewEntry(row[0], row[1], row[2], row[3], row[4], row[5], row[6], -5)
    except Exception as err:
      print(f"Exception trying to insert into DB! {err} ")


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


  def TryConnect(self, numRetries=10):
    """Checks if the mysql connection is connected. 
    If its not, retries until a connection is made (for numRetries attempts...)"""
    while not self._dbCNX or not self._dbCNX.is_connected():
        if self._dbCNX:
            print("Connection to DB failed, retrying after 10s...")
            time.sleep(10)  # delay 10 seconds and try again
        if self.__Connect() :
          break
        numRetries-=1
        if numRetries<=0 :
          break

    return self._dbCNX and self._dbCNX.is_connected()


  def GetMaxDate(self):
    """Return the max existing date in the DB"""
    if(not self.TryConnect(1)):  
      return None;

    myCursor=self._dbCNX.cursor()
    myCursor.execute("SELECT MAX(LocDate) AS MaxDate FROM LocationHistory")
    row = myCursor.fetchone()
    if row is not None:
      return row[0]
    else:
      return None


  def GetTracks(self,IDTracker,numLimit=20,fromData: datetime=date.today(), numRetries=10):
    fromData=fromData + timedelta(days=-1)
    if not self.TryConnect(numRetries):
        return None
    if (numLimit==0):
        numLimit=20
    sql="SELECT Latitude, Longitude, LocDate FROM LocationHistory WHERE IDTRACKER=%s AND LocDate>=%s ORDER BY IDTRACKER, LOCDATE DESC" #LIMIT %s"
    #mycursor = self._dbCNX.cursor()
    #df=pd.read_sql(sql, self._dbCNX, params=(IDTracker, fromData,numLimit))
    df=pd.read_sql(sql, self._dbCNX, params=(IDTracker, fromData))

    if df.empty:
        sql="SELECT Latitude, Longitude, LocDate FROM LocationHistory WHERE IDTRACKER=%s ORDER BY IDTRACKER, LOCDATE DESC LIMIT %s"
        df=pd.read_sql(sql, self._dbCNX, params=(IDTracker, numLimit))
    self._dbCNX.commit()
    #mycursor.close()
    return df
