import paho.mqtt.client as mqtt
import DbAccess as db
import DbAccessLite as dbLite
import Params
import time
from datetime import datetime

MQTTBROKER="192.168.1.140"
MQTTPORT=1888
TOPIC_DEBUG ="PChan/Debug"
TOPIC_BABYTRACKER = "babytracker/loc"

#dbuser=babytrack
#dbpass=@BabyTrack3r

_MainDbConnected=False
_MyDb=None
_MyDbLite=None

def PublishText(topic, msg):
	theMsg="["+str(datetime.now())+ "]"+ msg
	_myClient.publish(topic, theMsg, 0, True)


def UpdateMainDbFromLocal():
	"""Inserts the latests entries available in the local sqlite db into the main DB"""
	if(not _MyDbLite or not _MyDb.TryConnect(1)):
		return
	try:
		maxDate=_MyDb.GetMaxDate()
		missingRows=_MyDbLite.GetEntriesFromDate(maxDate)
		_MyDb.InsertEntries(missingRows)
	except Exception as err:
		print(f"Exception trying to insert {len(missingRows)} into DB!: {err}")



def OnMessageReceived(client, userdata, msg):
	""" The callback for when a PUBLISH message is received from the server.
	The message will be inserted in the DB if connection is available. """
	msg.payload = msg.payload.decode("utf-8")
	theMsg="Received message ["+msg.topic+"]["+str(msg.payload)+"]"
	print(theMsg)
	PublishText(TOPIC_DEBUG, theMsg)

	global _MainDbConnected

	#messages are like: "1, 1609515744, 41.472366, 2.043975, 0, 0.000000, 3.78"
	trackMsg=str(msg.payload).split(",")
	if len(trackMsg) >= 6: #we don't care if the insert fails....
		_MyDbLite.InsertNewEntry(trackMsg[0], datetime.fromtimestamp(int(trackMsg[1])), trackMsg[2], trackMsg[3], trackMsg[4], trackMsg[5], trackMsg[6])
		if(not _MainDbConnected and _MyDb.TryConnect()):
			_MainDbConnected=True
			UpdateMainDbFromLocal()

		_MyDb.InsertNewEntry(trackMsg[0], datetime.fromtimestamp(int(trackMsg[1])), trackMsg[2], trackMsg[3], trackMsg[4], trackMsg[5], trackMsg[6]) 
		
def Connect2Mqtt():
	"""Tries to connect to the mqtt broker, and waits if the connection is not possible"""
	myClient=mqtt.Client("PchanUpdater", False)
	myClient.on_message=OnMessageReceived

	isOk=False
	while not isOk:
		try:
			myClient.connect(MQTTBROKER, MQTTPORT) #"mitotoro.synology.me"
			myClient.subscribe(TOPIC_BABYTRACKER)
			isOk=True
		except:
			print("It was not possible to connect to the MQTT broker [",MQTTBROKER,":",MQTTPORT,"]. Waiting 10s...")
			time.sleep(10)

	return myClient


##########################
# MAIN PROGRAM STARTS HERE 😊
########################## 
print("Starting MQTT 2 DB updater...")
_myClient=Connect2Mqtt()
print("Mqtt client connected to broker [",MQTTBROKER,":",MQTTPORT,"] successfully! Topic=[",TOPIC_BABYTRACKER,"]")
PublishText(TOPIC_DEBUG, "Starting Python Updater")

_MyDb=db.BabyTrackerDB(Params.DB_USER, Params.DB_PASS, Params.DB_SERVER, Params.DB_DATABASE, Params.DB_PORT)
_MyDbLite=dbLite.DbAccessLite(Params.DBLITE_PATH)

_myClient.loop_forever()
