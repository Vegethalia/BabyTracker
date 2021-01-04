import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import DbAccess as db
import Params
import dash
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html


MAX_LATITUDE=41.4803
MIN_LATITUDE=41.4659
MAX_LONGITUDE=2.0699
MIN_LONGITUDE=2.0415

BBox = (MIN_LONGITUDE,   MAX_LONGITUDE, MIN_LATITUDE, MAX_LATITUDE)       
#Template Blootstrap
external_stylesheets = [dbc.themes.CYBORG] # [https://codepen.io/chriddyp/pen/bWLwgP.css']
app = dash.Dash(__name__, external_stylesheets=external_stylesheets)


##########################
# MAIN PROGRAM STARTS HERE 😊
########################## 
_MyDb=db.BabyTrackerDB(Params.DB_USER, Params.DB_PASS, Params.DB_SERVER, Params.DB_DATABASE, Params.DB_PORT)
mydfTracks=_MyDb.GetTracks(1)

ruh_m=plt.imread('E:\Projectes\Pleyha\mapStCugat.png')
fig, ax = plt.subplots(figsize = (10,6))
plt.plot(mydfTracks.Longitude, mydfTracks.Latitude, 'og--', lw=3)
ax.scatter(mydfTracks.Longitude, mydfTracks.Latitude, zorder=1, alpha= 0.2, c='b', s=520)
plt.plot(mydfTracks.Longitude,  mydfTracks.Latitude, 'Hy:', zorder=1, lw=3,alpha=0.5)
ax.set_title('Mapa Sant Cugat')
ax.set_xlim(BBox[0],BBox[1])
ax.set_ylim(BBox[2],BBox[3])

ax.imshow(ruh_m, zorder=0, extent = BBox, aspect= 'equal')
#plt.show()

figAux={
    'data': [
        {'x': [1, 2, 3], 'y': [4, 1, 2], 'type': 'bar', 'name': 'SF'},
        {'x': [1, 2, 3], 'y': [2, 4, 5], 'type': 'bar', 'name': u'Montréal'},
    ],
    'layout': {
        'title': 'Dash Data Visualization'
    }
}

cap=dbc.Jumbotron(fluid=True, className="pt-2 pb-2", children=[
    dbc.Row(children=[
		    html.H4("Benvingut!! Has perdut els babys???, Tranquil els trobarem!" ), 
		    ],  justify="center", align="center"),
])
app.layout=html.Div(children=[
	cap,
    dbc.Row(children=[
        dbc.Col(className="aborder border-primary", width=12, children=[
            dcc.Graph(id='BabyShowTrack', figure=figAux)
        ])
    ])
])

if __name__ == '__main__':
	app.run_server(debug=True, use_reloader=False, port=8051)