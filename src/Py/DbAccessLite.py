import sqlite3
from datetime import datetime

class DbAccessLite():
  """Basic class to store temporal BabyTracker info in a SQLITE database"""
  _DbFile=""
  _MyConn=None

  def __init__(self, dbPath):
    self._DbFile=dbPath

  def InsertNewEntry(self, idTracker, theDate, latitude, longitude, altitude, speed, battery):
    if self._MyConn==None and not self.__Connect():
      return False

    try:
      #theDate=datetime.fromtimestamp(int(theDate)) #, tzinfo=timezone('Etc/GMT+1')
      mycursor = self._MyConn.cursor()
      sql="INSERT INTO LocationHistory (IDTRACKER, LocDate, Latitude, Longitude, Altitude, Speed, Battery) VALUES (?, ?, ?, ?, ?, ?, ?)"
      values=(idTracker, theDate, latitude, longitude, altitude, speed, battery)
      mycursor.execute(sql, values)
      self._MyConn.commit()
    except Exception as err:
      print("Exception trying to insert into DB!: ", err)


  def GetEntriesFromDate(self, maxDate: datetime):
    """Returns all entries available in DB from the passed date"""
    if(self._MyConn is None and not self.__Connect()):
      return None
    if maxDate is None:
      return None
    myCursor=self._MyConn.cursor()
    myCursor.execute("SELECT IDTRACKER, LocDate, Latitude, Longitude, Altitude, Speed, Battery FROM LocationHistory WHERE LocDate>?", (str(maxDate),))
    return myCursor.fetchall()


  def __Connect(self):
    """Tries to connect to the sqlite db. Returns true or false"""
    try:
        self._MyConn = sqlite3.connect(self._DbFile)
    except Exception as err:
            print("Some Error occurred while connecting to SQLiteDB:", err)

    return self._MyConn is not None

    