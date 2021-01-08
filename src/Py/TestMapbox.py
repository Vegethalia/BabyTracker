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

MAX_LATITUDE=41.4803
MIN_LATITUDE=41.4659
MAX_LONGITUDE=2.0699
MIN_LONGITUDE=2.0415


#Template Blootstrap
external_stylesheets = [dbc.themes.CYBORG] # [https://codepen.io/chriddyp/pen/bWLwgP.css']
app = dash.Dash(__name__, external_stylesheets=external_stylesheets)

#Exemple amb plotly.express
_MyDb=db.BabyTrackerDB(Params.DB_USER, Params.DB_PASS, Params.DB_SERVER, Params.DB_DATABASE, Params.DB_PORT)
mydfTracks=_MyDb.GetTracks(1,2)
#mydfTracksA=_MyDb.GetTracks(1,2,datetime.strptime('03/01/2021','%d/%m/%Y'))
##fig = px.scatter_mapbox(mydfTracks,  lat=mydfTracks.Latitude, lon=mydfTracks.Longitude, color_discrete_sequence=["fuchsia"], zoom=18, height=800)
#fig = px.line_mapbox(mydfTracks, lat=mydfTracks.Latitude, lon=mydfTracks.Longitude, zoom=18, height=800)
#fig.update_layout(mapbox_style="open-street-map")
#fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0})

#Exemple amb plotly.graph_objects
import plotly.graph_objects as go


#fig = go.Figure(go.Scattermapbox(
#    mode = "markers+lines",
#    lon = mydfTracks.Longitude,
#    lat = mydfTracks.Latitude,
#    marker = {'size': 10, 'color':'blue'},
#    name='Dad'))

#fig.add_trace(go.Scattermapbox(
#    mode = "markers+lines",
#    lon = mydfTracksA.Longitude,
#    lat = mydfTracksA.Latitude,
#    marker = {'size': 10, 'color':'fuchsia'},
#    name='Arlet&Erola'))

#fig.update_traces(
#    marker=go.scattermapbox.Marker(
#        color='rgb(0, 255, 0)',
#        opacity=0.7,
#        sizeref=.5
#    ),
#     name='Arlet',
#    selector=dict(geo='geo')
#    )
#fig.update_layout(
#    margin ={'l':0,'t':0,'b':0,'r':0},
#    height=750,
#    mapbox = {
#        'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
#        'style': "stamen-terrain",
#        'zoom': 12},
#    title_text='Cercador de Babys')


cap=dbc.Jumbotron(fluid=True, className="pt-2 pb-2", children=[
    dbc.Row(children=[
		    html.H4("Benvingut!! Has perdut els babys???, Tranquil els trobarem!" ), 
		    ],  justify="center", align="center"),
])
app.layout=html.Div(children=[
	cap,
    dbc.Row(children=[
        dbc.Col(className="aborder border-primary", width=9, children=[
            dcc.Graph(id='idBabyShowMap')
        ]),
        dbc.Col(className="aborder border-primary", width=3, children=[
             html.Div(
                [
                    dbc.Button("Actualitza", id="idButton", className="mr-2"),
                    html.Span(id="idOutput", style={"vertical-align": "middle"}),
                    html.Span(id="idLocation", style={"vertical-align": "middle"})
                ]
            )
        ])
    ]),
    dbc.Row(children=[
        dbc.Col(width=10, children=[
            html.Div([
                   dcc.Slider(
                        id='idSlider',                        
                        min=5,
                        max=120,
                        value=0,
                        marks={str(i):str(i) for i in range(5,120,5)},
                        step=None,
                    )
                ])
            ])
        ]),
   html.Script( [ 'alert("Hello World!")'
   ])
])

#@app.callback(
#    Output("idBabyShowMap","figure"), [Input("idSlider","value")]
#)
#def update_Map_Slider(HourValue):
#    DateHour=date.today()+timedelta(Hours=-HourValue)
#    dfSelected=mydTracks[mydfTracks[LocDate]>=DateHour]
#    fig = go.Figure(go.Scattermapbox(
#    mode = "markers+lines",
#    lon = dfSelected.Longitude,
#    lat = dfSelected.Latitude,
#    marker = {'size': 10, 'color':'fuchsia'},
#    name='DadUpdated'))
#    fig.update_layout(
#    margin ={'l':0,'t':0,'b':0,'r':0},
#    height=750,
#    mapbox = {
#        'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
#        'style': "stamen-terrain",
#        'zoom': 12},
#    title_text='Cercador de Babys')
#    #mydfTracks=_MyDb.GetTracks(1,HourValue)
#    #fig = go.Figure(go.Scattermapbox(
#    #    mode = "markers+lines",
#    #    lon = mydfTracks.Longitude,
#    #    lat = mydfTracks.Latitude,
#    #    marker = {'size': 10, 'color':'fuchsia'},
#    #    name='DadUpdated'))
#    #fig.update_layout(
#    #    margin ={'l':0,'t':0,'b':0,'r':0},
#    #    height=750,
#    #    mapbox = {
#    #        'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
#    #        'style': "stamen-terrain",
#    #        'zoom': 12},
#    #    title_text='Cercador de Babys')
#    return(fig)

@app.callback(
    Output("idLocation", "children"), 
    Output("idBabyShowMap", "figure"),
    Input("idButton", "n_clicks")
)
def GetUserLocation(n):
    ipClient=(request.remote_addr)
    ipClient="93.176.164.33"
    myloc = geocoder.ip(ipClient)
      
    fig = go.Figure(go.Scattermapbox(
    mode = "markers",
    lon =[myloc.latlng[1]],
    lat = [myloc.latlng[0]],
    marker =dict(size=20, color='green'),
    name='UserLoc'))

    fig.update_layout(
    margin ={'l':0,'t':0,'b':0,'r':0},
    height=750,
    mapbox = {
        'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
        'style': "stamen-terrain",
        'zoom': 12},
    title_text='Cercador de Babys')
    return f"\nIP:{ipClient} -\n Latlong: {myloc.latlng}", fig         
          

@app.callback(
    Output("idOutput", "children"), [Input("idButton", "n_clicks")]
)
def on_button_click(n):
    if n is None:
        return "Not clicked."
    else:
        return f"Clicked {n} times."

#@app.callback(
#    Output("idBabyShowMap", "figure"), [Input("idButton", "n_clicks")]
#)
#def update_figuresd(n):
#    if n is None:
#        numTracks=2
#    else:
#        numTracks=n*10
#    mydfTracks=_MyDb.GetTracks(1,numTracks)
#    fig = go.Figure(go.Scattermapbox(
#        mode = "markers+lines",
#        lon = mydfTracks.Longitude,
#        lat = mydfTracks.Latitude,
#        marker = {'size': 10, 'color':'fuchsia'},
#        name='DadUpdated'))
#    fig.update_layout(
#        margin ={'l':0,'t':0,'b':0,'r':0},
#        height=750,
#        mapbox = {
#            'center': {'lon':(MIN_LONGITUDE+MAX_LONGITUDE)/2, 'lat': (MIN_LATITUDE+MAX_LATITUDE)/2},
#            'style': "stamen-terrain",
#            'zoom': 12},
#        title_text='Cercador de Babys')
#    return(fig)


if __name__ == '__main__':
	app.run_server(debug=True, use_reloader=False, port=8051)