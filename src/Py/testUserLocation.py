import dash
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import geocoder
from flask import request

external_stylesheets = [dbc.themes.CYBORG] # [https://codepen.io/chriddyp/pen/bWLwgP.css']
app = dash.Dash(__name__, external_stylesheets=external_stylesheets)


app.layout=html.Div(children=[
    html.Div(
    [
        dbc.Button("Actualitza", id="idButton", className="mr-2"),
        html.Span(id="idLocation", style={"vertical-align": "middle"})
    ])
])

@app.callback(
    Output("idLocation", "children"), [Input("idButton", "n_clicks")]
)
def on_buttonGetLocation_click(n):
    if n is None:
        return "Searching gps..."
    else:
        ipClient=(request.remote_addr)
        ipClient="93.176.164.33"
        myloc = geocoder.ip(ipClient)
        return f" IP: {ipClient} - Coordenades: {myloc.latlng}"        

if __name__ == '__main__':
	app.run_server(debug=True, use_reloader=False, port=8051)