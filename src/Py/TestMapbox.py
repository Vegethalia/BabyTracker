import pandas as pd
import dash
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import DbAccess as db
import Params
import plotly.express as px
from datetime import datetime
import geocoder
from flask import request
import plotly.graph_objects as go

MAX_LATITUDE=41.4803
MIN_LATITUDE=41.4659
MAX_LONGITUDE=2.0699
MIN_LONGITUDE=2.0415

#Template Blootstrap
external_stylesheets = [dbc.themes.CYBORG] # [https://codepen.io/chriddyp/pen/bWLwgP.css']
app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

MapBoxToken = open("resources/mapbox_token.txt").read()

#Exemple amb plotly.express
_MyDb=db.BabyTrackerDB(Params.DB_USER, Params.DB_PASS, Params.DB_SERVER, Params.DB_DATABASE, Params.DB_PORT)
mydfTracks=_MyDb.GetTracks(1,20)

#mydfTracksA=_MyDb.GetTracks(1,2,datetime.strptime('03/01/2021','%d/%m/%Y'))

#Dibuixem el mapa base amb la posició de l'usuari      
fig = go.Figure(go.Scattermapbox(
mode = "markers",
lon =[MAX_LONGITUDE],
lat = [MAX_LATITUDE],
marker =dict(size=20, color='green'),
name='UserLoc'))

fig.update_layout(
margin ={'l':0,'t':0,'b':0,'r':0},
height=750,
mapbox = {
    'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
    'accesstoken':MapBoxToken,
    'style': 'stamen-watercolor',
    'zoom': 12},
title_text='Cercador de Babys')

cap=dbc.Jumbotron(fluid=True, className="pt-2 pb-2", children=[
    dbc.Row(children=[
		    html.H4("Benvingut!! Has perdut els babys???, Tranquil els trobarem!" ), 
		    ],  justify="center", align="center"),
])
app.layout=html.Div(children=[
	cap,
    dbc.Row(children=[
            dbc.Col(className="aborder border-primary", width=4,  children=[
                 html.Div([
                    'Tipus de Mapa',
                    dcc.Dropdown(
                    id='idTipusMap',
                    options=[{'label': i, 'value': i} for i in ["basic","dark","light","open-street-map","outdoors","satellite","satellite-streets","stamen-terrain","stamen-watercolor","streets"]],
                    value='stamen-watercolor',
                    clearable=False
                    ),
                ])
            ]),
             dbc.Col(className="aborder border-primary", width=6, children=[
                html.Div([
                    dbc.Button("Actualitza", id="idButton", className="mr-2"),
                    html.Span(id="idOutput", style={"vertical-align": "middle"}),
                    html.Span(id="idLocation", style={"vertical-align": "middle"})
                ]) 
          ])
        ]),

    dbc.Row(children=[
        dbc.Col(width=10, children=[
            html.Div([
                   'Seleccionar últims minuts',
                   dcc.Slider(
                        id='idSlider',                        
                        min=5,
                        max=120,
                       step=None,
                        marks={str(i):{'label':str(i)+'m'} for i in range(5,125,5)},
                        value=5                        
                    )
                ],style={'height':75})
            ])
        ]),
    dbc.Row(children=[
        dbc.Col(className="aborder border-primary", width=10, children=[
            dcc.Graph(id='idBabyShowMap')
        ])
    ])
])

#dibuixem tot el mapa amb el que es necessita. tipus de mapa, loc de l'usuari i llista de punts
@app.callback(
    Output("idLocation", "children"), 
    Output("idBabyShowMap", "figure"),
    Input("idTipusMap","value"),
    Input("idSlider","value")
)
def update_MapTotal(TipusMap, TempsSelected):
    ipClient=(request.remote_addr)
    ipClient="93.176.164.33"
    myloc = geocoder.ip(ipClient)
      
    fig = go.Figure(go.Scattermapbox(
    mode = "markers+lines",
    lon =[myloc.latlng[1]],
    lat = [myloc.latlng[0]],
    textposition='top right',
    textfont=dict(size=16, color='black'),
    text=["TEXTING"],
    marker =dict(size=20, color='green'),
    name='UserLoc'))


    #DateHour=date.today()+timedelta(minutes=-TempsSelected)
    #dfSelected=mydfTracks[mydfTracks[LocDate]>=TempsSelected]
    dfSelected=mydfTracks
    fig.add_trace(go.Scattermapbox(
        mode = "markers+lines+text",
        lon = dfSelected.Longitude,
        lat = dfSelected.Latitude,
        textposition='top right',
        textfont=dict(size=16, color='black'),
        text=dfSelected.LocDate,
        marker = {'size': 10, 'color':'fuchsia'},
        name='Arlet&Erola'))
    
    fig.update_layout(
    margin ={'l':0,'t':0,'b':0,'r':0},
    height=750,
    mapbox = {
        'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
        'accesstoken':MapBoxToken,
        'style': TipusMap,
        'zoom': 12},
    title_text='Cercador de Babys')
    #mydfTracks=_MyDb.GetTracks(1,HourValue)
    #fig = go.Figure(go.Scattermapbox(
    #    mode = "markers+lines",
    #    lon = mydfTracks.Longitude,
    #    lat = mydfTracks.Latitude,
    #    marker = {'size': 10, 'color':'fuchsia'},
    #    name='DadUpdated'))
    #fig.update_layout(
    #    margin ={'l':0,'t':0,'b':0,'r':0},
    #    height=750,
    #    mapbox = {
    #        'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
    #        'style': "stamen-terrain",
    #        'zoom': 12},
    #    title_text='Cercador de Babys')
    return  f"\nIP:{ipClient} -\n Latlong: {myloc.latlng} --Slider SELECT: {TempsSelected}",fig


#Modifiquem el tipus de mapa segons el que hagi escollit l'usuari al dropdownlist
@app.callback(
    Output("idBabyShowMap","figure"), Input("idTipusMap","value")
)
def update_TipusMap(TipusMap):
 fig.update_layout(
    margin ={'l':0,'t':0,'b':0,'r':0},
    height=750,
    mapbox = {
        'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
        'accesstoken':MapBoxToken,
        'style': TipusMap,
        'zoom': 12},
    title_text='Cercador de Babys')
 return(fig)

if __name__ == '__main__':
	app.run_server(debug=True, use_reloader=False, port=8051)