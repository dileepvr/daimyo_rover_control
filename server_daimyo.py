import socket
import select
import getopt
import sys
import os
import logging
import threading
import json
import datetime
import numpy as np
from functools import partial
from daimyo_utils import Rover, st_dict

# For rendering web application/page
from bokeh.models import Select, Button, TextInput, Div, Diamond
from bokeh.models import ColumnDataSource, Triangle, RadioButtonGroup
from bokeh.models import Panel, Tabs, DataTable, TableColumn, NumberEditor
from bokeh.models import NumberFormatter, StringFormatter, CircleCross
from bokeh.models import CheckboxButtonGroup, CheckboxGroup
from bokeh.plotting import figure
from bokeh.models.tools import HoverTool, PointDrawTool, PolyDrawTool
from bokeh.models.tools import PolyEditTool, TapTool
from bokeh.events import Tap, DoubleTap
from bokeh.layouts import column, row, Spacer

# For web server backend
from bokeh.server.server import Server
from jinja2 import Environment, FileSystemLoader
from tornado.web import RequestHandler
from tornado.web import StaticFileHandler
from bokeh.embed import server_document
import asyncio


mainlock = threading.RLock()
if not os.path.exists('maps'):  # Store map (json) files here
    os.mkdir('maps')
if not os.path.exists('sequences'):  # Store command sequences and patrols
    os.mkdir('sequences')
if not os.path.exists('datalogs'):  # Store log (info) output
    os.mkdir('datalogs')
ddate = datetime.datetime.now()
subfolder = 'datalogs/'+ddate.strftime('%Y_%m_%d')
if not os.path.isdir(subfolder):  # Store logs for the day
    os.mkdir(subfolder)
logfilename = subfolder+'/'+ddate.strftime('serverlog_%H_%M_%S.log')

env = Environment(loader=FileSystemLoader('web_templates'))


class IndexHandler(RequestHandler):
    def get(self):
        template = env.get_template('embed.html')
        script = server_document('http://localhost:5006/webapp')
        self.write(template.render(script=script, template="Tornado"))


def check_sane_str(instr):
    instr = str(instr)
    if any(char in instr for char in ['/', '*', '..']):
        print("Illegal character found in %s." % instr)
        return False
    elif not instr:
        return False
    else:
        return True


def webapp(doc):

    global updatecmd  # Integer for next_tick_callback to update DataSources
    global eventx, eventy
    global list_of_rovers, roverlistchanged, roverchangetype
    global webroverlist, rover_indx, rovermenu, seqdict
    global infodict, infofmt, data_s, data_seq, seqdict
    # global weblog
    updatecmd = 0
    [eventx, eventy] = [0, 0]
    webroverlist = dict([('name', []), ('version', []), ('hflag', []),
                         ('x', []), ('y', []), ('angle', []),
                         ('state', []), ('loop', []), ('lflag', []),
                         ('seqfile', []), ('seqlist', []),
                         ('xhist', []), ('yhist', []), ('threadname', [])])
    # emptyroverlist = webroverlist
    data_s = ColumnDataSource(webroverlist)
    rover_indx = -1  # index if rover currently selected

    # Next-tick callback for updating ColumnDataSources
    # Bokeh recommends updating all ColumnDataSources in one function call
    def update_CDS():
        global updatecmd, eventx, eventy
        global data_s, webroverlist, data_seq, seqdict
        global dnorth, dobs, dwalls, drfid, mapread
        global fcmdsource, gotodict, setposdict, turndict
        global goto_dest, tempglobdict
        try:
            if updatecmd == 0:
                data_s.data = dict(webroverlist)
                data_seq.data = dict(seqdict)
            elif updatecmd == 1:
                dnorth.data = mapread[0]
                dobs.data = mapread[1]
                dwalls.data = mapread[2]
                drfid.data = mapread[3]
            elif updatecmd == 2:
                fcmdsource.data = dict(gotodict)
                goto_dest.data = dict(x=[eventx], y=[eventy])
            elif updatecmd == 3:
                fcmdsource.data = dict(setposdict)
                goto_dest.data = dict(x=[eventx], y=[eventy])
            elif updatecmd == 4:
                fcmdsource.data = dict(turndict)
                goto_dest.data = dict(x=[eventx], y=[eventy])
            elif updatecmd == 5:
                goto_dest.data = dict(x=[eventx], y=[eventy])
                fcmdsource.data = dict(gotodict)
                fields = [str(i) if i != None else ''
                          for i in dict(fcmdsource.data)['values']]
                send_fcmd(cmdstring='GOTO', fields=fields)
            elif updatecmd == 6:
                goto_dest.data = dict(x=[eventx], y=[eventy])
                fcmdsource.data = setposdict
                fields = [str(i) if i != None else ''
                          for i in dict(fcmdsource.data)['values']]
                send_fcmd(cmdstring='SETPOS', fields=fields)
            elif updatecmd == 7:
                goto_dest.data = dict(x=[eventx], y=[eventy])
                fcmdsource.data = turndict
                fields = [str(i) if i != None else ''
                          for i in dict(fcmdsource.data)['values']]
                send_fcmd(cmdstring='TURN', fields=fields)
            elif updatecmd == 8:
                valdict = dict(fcmdsource.data)
                goto_dest.data = dict(x=[valdict['values'][0]],
                                      y=[valdict['values'][1]])
            elif updatecmd == 9:
                fcmdsource.data = dict(tempglobdict)
            if updatecmd == 10:
                data_seq.data = dict(seqdict)
        except RuntimeError:  # Happens if you reload webpage when running
            pass

    def append_data_s(rover):
        global data_s, webroverlist
        rover.lock.acquire()
        webroverlist['version'].append(rover.version)
        webroverlist['x'].append(rover.x)
        webroverlist['y'].append(rover.y)
        webroverlist['angle'].append(rover.angle)
        webroverlist['state'].append(st_dict[rover.state])
        webroverlist['loop'].append(rover.superstate)
        webroverlist['hflag'].append(rover.heartbeat)
        webroverlist['lflag'].append(rover.loopflag)
        webroverlist['seqfile'].append(rover.seqfile)
        webroverlist['seqlist'].append(rover.seqlist)
        webroverlist['xhist'].append([rover.x])
        webroverlist['yhist'].append([rover.y])
        webroverlist['threadname'].append(rover.thread.getName())
        rover.lock.release()
        webroverlist['angle'] = [x-90 for x in webroverlist['angle']]

    for rover in list_of_rovers:
        rover.lock.acquire()
        _tmpvar = rover.name+'_____('+rover.thread.getName()+')'
        webroverlist['name'].append(_tmpvar)
        append_data_s(rover)
        rover.lock.release()
    data_s.data = dict(webroverlist)
    numrovs = len(webroverlist['name'])
    mainlock.acquire()
    roverlistchanged = False
    mainlock.release()

    # map variables
    global dnorth, dobs, dwalls, drfid, mapread
    mapread = [{'ang': [90]},
               {'x': [], 'y': []},
               {'x': [[]], 'y': [[]]},
               {'x': [], 'y': [], 'name': []}]
    dnorth = ColumnDataSource(mapread[0])
    dobs = ColumnDataSource(mapread[1])
    dwalls = ColumnDataSource(mapread[2])
    drfid = ColumnDataSource(mapread[3])

    # Three panel widths
    [left_w, center_w, right_w] = [400, 1200, 200]

    # List of live rovers (left panel)
    rovermenu = Select(title="Live Rovers: %d" % numrovs,
                       options=webroverlist['name'],
                       value='', width=150, align='start')
    infofmt = """
    Name: {name}   (ver: {version})<br />
    (x, y): ({x:.3f}, {y:.3f}) m<br />
    angle: {ang:.1f} deg.<br />
    state: {state} <br/>seq_indx: {loop}, loop: {lflag}"""
    infodict = dict([('name', 'sam'), ('x', 0.0), ('y', 0.0), ('version', '0'),
                     ('ang', 90), ('state', 'st_IDLE'), ('loop', -1),
                     ('lflag', False)])
    roverstate = Div(text='''''', width=left_w-160, height=120,
                     background="whitesmoke", align='end',
                     style=dict([("font-weight", "bold"),
                                 ("font-size", "large"),
                                 ("color", "DarkRed"),
                                 ("opacity", 1.0)]))

    # Compass indicator (right panel)
    fc = figure(x_range=(-1, 1), y_range=(-1, 1), title="Compass",
                plot_width=100, plot_height=100, tools='',
                align='start', output_backend="webgl")
    fc.title.text_font_size = '14pt'
    fc.margin = (0, 0, 0, 15)
    fc.xaxis.visible = False
    fc.xgrid.visible = False
    fc.yaxis.visible = False
    fc.ygrid.visible = False
    fc.toolbar.logo = None
    fct = fc.triangle(x=0, y=0, angle=dnorth.data['ang'][0] - 90,
                      size=50,
                      angle_units="deg")
    fcd = fc.diamond(x=0, y=0, angle=dnorth.data['ang'][0] - 90,
                     size=50,
                     angle_units="deg")
    compasscol = [
        TableColumn(field="ang", title="North (deg.)",
                    editor=NumberEditor(step=0.1),
                    formatter=NumberFormatter(font_style="bold",
                                              format='000.0'))
    ]
    fctxt = DataTable(source=dnorth, height=100, columns=compasscol,
                      width=110, align='end', editable=False,
                      index_position=None)

    # Show/hide history and map (right panel)
    showhide = CheckboxButtonGroup(labels=['show trails', 'show map'],
                                   active=[0, 1], width=right_w,
                                   margin=[20, 0, 20, 0])

    # Map manipulation panel
    mapfilein = TextInput(value="test_map.json",
                          title="map (*.json) file:",
                          align='center',
                          width=right_w, height=50)
    LOADmapbtn = Button(label="load map", height=40, width=right_w,
                        align='center', button_type='default',
                        margin=[60, 0, 0, 0])
    EDITmapbox = CheckboxGroup(labels=['Enable map edit'], active=[],
                               width=right_w, align='center',
                               margin=[100, 0, 0, 0])
    SAVEmapbtn = Button(label="save map", height=40, width=right_w,
                        align='center', button_type='default')
    maptab = Panel(child=column([EDITmapbox, mapfilein, LOADmapbtn,
                                 SAVEmapbtn]),
                   title="Map panel")
    rfidcol = [
        TableColumn(field="x", title="x (m)",
                    editor=NumberEditor(step=0.1), sortable=False,
                    formatter=NumberFormatter(font_style="bold",
                                              format='00.0')),
        TableColumn(field="y", title="y (m)",
                    editor=NumberEditor(step=0.1), sortable=False,
                    formatter=NumberFormatter(font_style="bold",
                                              format='00.0')),
        TableColumn(field="name", title="RFID UID", sortable=False,
                    formatter=StringFormatter(font_style="bold"))
    ]
    rfidtable = DataTable(source=drfid, columns=rfidcol,
                          height=300, editable=False, width=right_w+50,
                          reorderable=False, row_height=35,
                          index_position=None)
    rfidtab = Panel(child=rfidtable, title="Map RFID list")
    maptabs = Tabs(tabs=[maptab, rfidtab], visible=True, align='center')
    warntxt = Div(text='''File Exits<br /> Overwrite?''',
                  width=right_w, height=100,
                  align='center', style=dict([("font-weight", "bold"),
                                              ("font-size", "large"),
                                              ("color", "DarkRed"),
                                              ("opacity", 1.0)]))
    YESbtn = Button(label="YES", height=50, width=right_w, align='center',
                    button_type='success')
    CANCELbtn = Button(label="CANCEL", height=50, width=right_w,
                       align='center', button_type='danger')
    warntab = Panel(child=column([warntxt, YESbtn, CANCELbtn]),
                    title='')
    warntabs = Tabs(tabs=[warntab], visible=False, align='center')

    # Buttons for commands without fields (left panel)
    HALTallbtn = Button(label="HALT ALL", height=40, width=left_w,
                        button_type="warning")

    btn_w = int((left_w - 20)/3)
    HEARTallbtn = Button(label="HEARTBEAT ALL", height=40,
                         width=int(left_w/2)-5, align='start',
                         button_type="default")
    SILENTallbtn = Button(label="SILENT ALL", height=40,
                          width=int(left_w/2)-5, align='end',
                          button_type="default")
    PAUSEallbtn = Button(label="PAUSE ALL", height=40,
                         width=int(left_w/2)-5, align='start',
                         button_type="default")
    UNPAUSEallbtn = Button(label="UNPAUSE ALL", height=40,
                           width=int(left_w/2)-5, align='end',
                           button_type="default")
    IDbtn = Button(label="ID", height=40, width=btn_w,
                   button_type="default")
    POSbtn = Button(label="POS", height=40, width=btn_w,
                    button_type="default")
    HALTbtn = Button(label="HALT", height=40, width=btn_w,
                     button_type="warning")
    DIEbtn = Button(label="DIE", height=40, width=btn_w,
                    button_type="danger")
    nofieldbtngrid = row([IDbtn, POSbtn, HALTbtn])
    HEARTbtn = RadioButtonGroup(labels=["HEARTBEAT", "SILENT"],
                                active=1,
                                width=left_w)

    cmdmenu = ["SETPOS", "FWD", "BWD", "CFWD", "CBWD", "TURN", "ATURN",
               "CTURN", "GOTO", "OBS", "POBS", "SEARCH"]

    dropdown = Select(title="Select command", value=cmdmenu[0],
                      options=cmdmenu, width=200, align='center')

    global setposdict, fwddict, bwddict, cfwddict, cbwddict, turndict
    global aturndict, cturndict, gotodict, obsdict, pobsdict, searchdict
    global fcmdsource, tempglobdict
    setposdict = dict([('names', ['x (m)', 'y (m)', '[angle (deg.)]']),
                       ('values', [0.0, 0.0, 0.0])])
    fwddict = dict([('names', ['distance (m)', '[speed (m/s)]',
                               '[timeout (s)]']),
                    ('values', [0.1, 0.2, None])])
    bwddict = dict([('names', ['distance (m)', '[speed (m/s)]',
                               '[timeout (s)]']),
                    ('values', [0.1, 0.2, None])])
    cfwddict = dict([('names', ['[speed (m/s)]', '[timeout (s)]']),
                     ('values', [0.2, None])])
    cbwddict = dict([('names', ['[speed (m/s)]', '[timeout (s)]']),
                     ('values', [0.2, None])])
    turndict = dict([('names', ['angle (deg.)', '[timeout (s)]']),
                     ('values', [45.0, None])])
    aturndict = dict([('names', ['direction (0/1)', 'angle (deg.)',
                                 '[timeout (s)]']),
                      ('values', [0, 30.0, None])])
    cturndict = dict([('names', ['direction (0/1)', '[timeout (s)]']),
                      ('values', [0, None])])
    gotodict = dict([('names', ['x (m)', 'y (m)', '[speed (m/s)]',
                                '[end angle (deg.)]', '[timeout (s)]']),
                     ('values', [0.0, 0.0, 0.1, None, None])])
    obsdict = dict([('names', ['[angle (deg.)]', '[timeout (s)]']),
                    ('values', [180.0, None])])
    pobsdict = dict([('names', ['distance (m)', '[angle (deg.)]',
                                '[timeout (s)]']),
                     ('values', [0.5, -45.0, None])])
    searchdict = dict([('names', ['[radius (m)]', '[RFID tag UID]',
                                  'timeout (s)']),
                       ('values', [0.01, None, None])])

    fcmdsource = ColumnDataSource(setposdict)
    tempglobdict = {}

    # Table of fields
    fieldcols = [
        TableColumn(field="names", title="Field names",
                    formatter=StringFormatter(font_style="bold")),
        TableColumn(field="values", title="Values",
                    formatter=StringFormatter(font_style="bold"))
    ]
    tabletitle = Div(text='''Set Position''', width=left_w-20, height=30,
                     align='center')
    cmdfieldstable = DataTable(source=fcmdsource, columns=fieldcols,
                               width=left_w-20, height=300, editable=True,
                               align='center',
                               reorderable=False, row_height=35)
    tblset = column([tabletitle, cmdfieldstable])

    SENDbtn = Button(label="SEND", height=40, width=200,
                     button_type="success", align='center')
    cmdtab = Panel(child=column([SENDbtn, dropdown, tblset]),
                   title="Single Commands")
    # Sequences tab
    PAUSEbtn = Button(label="PAUSE", height=40,
                      width=btn_w, align='start',
                      button_type="default")
    UNPAUSEbtn = Button(label="UNPAUSE", height=40,
                        width=btn_w, align='end',
                        button_type="default")
    ADDROWseqbtn = Button(label="Add step", height=40,
                          width=btn_w, align='end',
                          button_type="default")
    READseqbtn = Button(label="Read", height=40, width=btn_w,
                        button_type="default")
    SAVEseqbtn = Button(label="Save", height=40, width=btn_w,
                        button_type="default")
    RUNseqbtn = Button(label="Load & Run", height=40, width=btn_w,
                       button_type="primary")
    seqfilein = TextInput(value="test_wedge.seq",
                          title="sequence (*.seq) file:",
                          align='center', margin=[10, 0, 20, 0],
                          width=left_w, height=50)
    seqcol = [
        TableColumn(field="lines", title="Steps",
                    formatter=StringFormatter(font_style="bold")),
    ]
    seqdict = {"lines": []}
    data_seq = ColumnDataSource(seqdict)
    seqtable = DataTable(source=data_seq, columns=seqcol,
                         width=left_w, height=200, editable=True,
                         reorderable=True, row_height=35)

    seqtab = Panel(child=column([row([PAUSEbtn, UNPAUSEbtn,
                                      READseqbtn]),
                                 row([ADDROWseqbtn, SAVEseqbtn,
                                      RUNseqbtn]), seqfilein,
                                 seqtable]),
                   title="Command sequences")
    rovtabs = Tabs(tabs=[cmdtab, seqtab], align='center')

    # Central log box below plot
    logbox = Div(text="""""", width=center_w,
                 background="whitesmoke",
                 style=dict([("font-weight", "bold"),
                             ("font-size", "large"),
                             ("color", "Green"),
                             ("opacity", 1.0)]))

    def fader():
        if logbox.style['opacity'] > 1.0:
            logbox.style['opacity'] -= 1.0
            logbox.text = logbox.text.split('<br />', 1)[1]
            doc.add_timeout_callback(fader, 10000)
        else:
            logbox.text = ''

    def set_logbox(text="""""", color="Green", fade=True):
        # weblog.debug(text)
        logbox.text += text+'<br />'
        logbox.style['opacity'] += 1.0  # Use as a counter
        if logbox.style['opacity'] > 3.0:
            logbox.style['opacity'] = 3.0
            logbox.text = logbox.text.split('<br />', 1)[1]
        logbox.style['color'] = color
        if fade:
            doc.add_timeout_callback(fader, 10000)

    def SAVEmapbtn_handler():
        global dnorth, dobs, dwalls, drfid
        mfile = mapfilein.value
        if check_sane_str(mfile):
            mapfile = 'maps/' + mfile
            if os.path.exists(mapfile):
                warntabs.visible = True
                showhide.disabled = True
                mapfilein.disabled = True
                LOADmapbtn.disabled = True
                SAVEmapbtn.disabled = True
            else:
                mapread = [dict(dnorth.data), dict(dobs.data),
                           dict(dwalls.data), dict(drfid.data)]
                fp = open(mapfile, 'w')
                json.dump(mapread, fp)
                fp.close()
                set_logbox(text="Map saved to %s." % mapfile)
        else:
            set_logbox(text="No {'*', '/', '..'} or blank string allowed!",
                       color='red')
            return

    def load_mapfile(mfile):
        global dnorth, dobs, dwalls, drfid, mapread
        global updatecmd
        if check_sane_str(mfile):
            mapfile = 'maps/' + mfile
            if os.path.exists(mapfile):
                fp = open(mapfile, 'r')
                mapread = json.load(fp)
                fp.close()
                updatecmd = 1
                doc.add_next_tick_callback(update_CDS)
                set_logbox(text="Map file %s loaded." % mapfile)
                return
            else:
                set_logbox(text="Map file %s does not exist!" % mapfile,
                           color='red')
                return
        else:
            set_logbox(text="No {'*', '/', '..'} or blank string allowed!",
                       color='red')
            return

    # Central map plotting area (center panel)
    mytools = "crosshair,pan,wheel_zoom,box_zoom,reset, \
    box_select,lasso_select"
    mapp = figure(output_backend="webgl", x_range=(-2, 2),
                  y_range=(-2, 2),
                  plot_width=center_w, plot_height=800, tools=mytools,
                  active_drag='pan', active_scroll='wheel_zoom')
    mapp.toolbar.active_inspect = None
    # mapp.toolbar.active_tap = None
    mapp.toolbar.logo = None
    mapp.xaxis.axis_label = "x-position (meters)"
    mapp.xaxis.axis_label_text_font_size = '16pt'
    mapp.xaxis.major_label_text_font_size = '16pt'
    mapp.yaxis.major_label_text_font_size = '16pt'
    mapp.yaxis.axis_label = "y-position (meters)"
    mapp.yaxis.axis_label_text_font_size = '16pt'
    hist_lines = mapp.multi_line(xs='xhist', ys='yhist',
                                 source=data_s, visible=True,
                                 color="red", alpha=0.8,
                                 line_width=2, line_dash=[4, 4])
    rover_triangles = mapp.triangle(x='x', y='y', angle='angle',
                                    source=data_s, fill_color=None,
                                    line_width=2, line_color="green",
                                    size=50, angle_units="deg")
    selected_glyph = Diamond(fill_alpha=1, fill_color="firebrick",
                             line_color=None)
    nonselected_glyph = Diamond(fill_alpha=0.2, fill_color="green",
                                line_color="firebrick")
    rover_diamonds = mapp.diamond(x='x', y='y', angle='angle',
                                  source=data_s, angle_units="deg",
                                  size=50, fill_alpha=0.2,
                                  fill_color="green")
    rover_diamonds.selection_glyph = selected_glyph
    rover_diamonds.nonselection_glyph = nonselected_glyph
    selected_glyph = Triangle(line_alpha=1, fill_color=None,
                              line_width=2, line_color="green")
    nonselected_glyph = Triangle(line_alpha=1, fill_color=None,
                                 line_width=1, line_color="green")
    rover_triangles.selection_glyph = selected_glyph
    rover_triangles.nonselection_glyph = nonselected_glyph
    mytaptool = TapTool(renderers=[rover_diamonds])
    mapp.add_tools(mytaptool)

    # Map display
    obstacles = mapp.circle(x='x', y='y', size=50, color="gray",
                            source=dobs, visible=True)
    obstacles.nonselection_glyph.fill_alpha = 0.5
    obstacles.nonselection_glyph.line_alpha = 0.5
    walls = mapp.multi_line(xs='x', ys='y', color='gray', source=dwalls,
                            line_width=4, visible=True)
    walls.nonselection_glyph.line_alpha = 0.5
    rfidtags = mapp.diamond(x='x', y='y', source=drfid, size=50,
                            fill_color='orange', line_color='black',
                            line_width=2, visible=True)
    rfidtags.nonselection_glyph.fill_alpha = 0.5
    rfidtags.nonselection_glyph.line_alpha = 0.5
    mapp.add_tools(HoverTool(tooltips=[("(x, y)", "(@x, @y)"),
                                       ("name", " @name")],
                             renderers=[rfidtags, rover_diamonds]))

    # Some plot tools
    obs_edit_tool = PointDrawTool(renderers=[obstacles],
                                  custom_tooltip='Add/Drag/Delete Map Circles',
                                  add=False, drag=False)
    rfid_edit_tool = PointDrawTool(renderers=[rfidtags],
                                   custom_tooltip='Add/Drag/Delete RFID tags',
                                   add=False, drag=False)
    vertex_rend_circ = mapp.circle([], [], size=10, color='red')
    wall_edit_tool = PolyEditTool(renderers=[walls],
                                  vertex_renderer=vertex_rend_circ,
                                  custom_tooltip='Edit map walls')
    wall_draw_tool = PolyDrawTool(renderers=[walls],
                                  vertex_renderer=vertex_rend_circ,
                                  custom_tooltip='Draw map walls')
    mapp.add_tools(obs_edit_tool)
    mapp.add_tools(rfid_edit_tool)
    mapp.add_tools(wall_draw_tool)
    mapp.add_tools(wall_edit_tool)

    # GOTO indicator
    global goto_dest
    goto_dest = ColumnDataSource(dict(x=[0.0], y=[0.0]))
    goto_glyph = CircleCross(x='x', y='y', fill_color=None,
                             line_width=1, line_color="mediumblue",
                             size=20, line_alpha=1.0)
    mapp.add_glyph(goto_dest, goto_glyph)

    # Compute angle given starting point and destination XY
    def compute_angle(xi=0, yi=0, xf=1, yf=1):
        if abs(xf - xi) > 0.0:
            tanarg = (yf-yi)/(xf-xi)
            destang = np.degrees(np.arctan(tanarg))
            if yf >= yi and xf < xi:
                destang += 180
            elif yf <= yi and xf < xi:
                destang += 180
            destang = destang % 360
        else:
            if yf > yi:
                destang = 0.0
            else:
                destang = 180.0
        return destang

    # Compass North table callback
    def fcsl_callback(attr, old, new):
        fct.glyph.angle = new['ang'][0]-90
        fcd.glyph.angle = new['ang'][0]-90

    # roverstate set
    def set_roverstate(rover):
        global infodict, infofmt
        infodict['name'] = rover.name
        infodict['version'] = rover.version
        [infodict['x'], infodict['y']] = [rover.x, rover.y]
        infodict['ang'] = rover.angle
        infodict['state'] = st_dict[rover.state]
        infodict['loop'] = rover.superstate
        infodict['lflag'] = rover.loopflag
        roverstate.text = infofmt.format(**infodict)
        roverstate.style['color'] = 'DarkRed'

    if numrovs > 0:  # Do this at startup
        set_roverstate(list_of_rovers[0])
        rover_indx = 0

    # Periodic update
    def update():
        global roverlistchanged, roverchangetype, rover_indx
        global data_s, rovermenu, updatecmd
        if roverlistchanged:
            mainlock.acquire()
            roverlistchanged = False
            if roverchangetype == 'NEW':  # New rover has joined
                rover = list_of_rovers[-1]
                rover.lock.acquire()
                _tmpvar = rover.name+'_____('+rover.thread.getName()+')'
                set_logbox(text="New rover %s detected." % _tmpvar)
                webroverlist['name'].append(_tmpvar)
                append_data_s(rover)
                rover.lock.release()
                updatecmd = 0
                doc.add_next_tick_callback(update_CDS)
                if len(webroverlist['name']) == 1:
                    rover_indx = 0
                    set_roverstate(rover)
                rovermenu.options = []  # Seems to be necessary for update
                rovermenu.options = webroverlist['name']
                rover_indx = 0 if rover_indx == -1 else rover_indx
                if rover_indx == len(list_of_rovers)-1:
                    set_roverstate(rover)
                rovermenu.title = 'Live Rovers: %d' % len(webroverlist['name'])
            elif roverchangetype[0:4] == 'DIE:':  # A rover has died
                rovermenu.options = []  # Seems to be necessary for update
                tempindx = int(roverchangetype[4:])
                set_logbox(text="Rover %s disconnected." %
                           webroverlist['name'][tempindx], color="Red")
                for key in webroverlist.keys():
                    del webroverlist[key][tempindx]
                updatecmd = 0
                doc.add_next_tick_callback(update_CDS)
                if webroverlist['name']:
                    rovermenu.options = webroverlist['name']
                else:
                    roverstate.text = ''''''
                    seqfilein.value = ''
                    seqdict["lines"] = []
                updatecmd = 0
                doc.add_next_tick_callback(update_CDS)
                if rovermenu.value in webroverlist['name']:
                    rover_indx = webroverlist['name'].index(rovermenu.value)
                else:
                    rover_indx = len(webroverlist['name'])-1
                    if rover_indx > -1:
                        rovermenu.value = rovermenu.options[rover_indx]
                rovermenu.title = 'Live Rovers: %d' % len(webroverlist['name'])
            mainlock.release()
        tempindx = -1
        for rover in list_of_rovers:  # Look for messages passed to webUI
            tempindx += 1
            rover.lock.acquire()
            if rover.wflag:
                rover.wflag = False
                if rover.wlist[0] == 'MYID':  # ID change
                    _tmpvar = rover.name+'_____('+rover.thread.getName()+')'
                    set_logbox(text="Rover %s changed ID to %s." %
                               (webroverlist['name'][tempindx], _tmpvar))
                    webroverlist['name'][tempindx] = _tmpvar
                    webroverlist['version'][tempindx] = rover.version
                    rovermenu.options = []  # Seems to be necessary for update
                    rovermenu.options = webroverlist['name']
                    if rover_indx == tempindx:
                        rovermenu.value = _tmpvar
                        set_roverstate(rover)
                elif rover.wlist[0] == 'MYPOS':  # Position change
                    webroverlist['x'][tempindx] = rover.wlist[1]
                    webroverlist['y'][tempindx] = rover.wlist[2]
                    webroverlist['angle'][tempindx] = rover.wlist[3]-90
                    if len(webroverlist['xhist'][tempindx]) == 1:
                        if [webroverlist['xhist'][tempindx][0],
                            webroverlist['yhist'][tempindx][0]] == [0, 0]:
                            webroverlist[
                                'xhist'][tempindx][0] = rover.wlist[1]
                            webroverlist[
                                'yhist'][tempindx][0] = rover.wlist[2]
                    if [webroverlist['xhist'][tempindx][-1],
                        webroverlist['yhist'][tempindx][-1]] != [
                            rover.wlist[1],
                            rover.wlist[2]]:
                        webroverlist['xhist'][tempindx].append(
                            rover.wlist[1])
                        webroverlist['yhist'][tempindx].append(
                            rover.wlist[2])
                        if len(webroverlist['xhist'][tempindx]) > 100:
                            webroverlist['xhist'][tempindx].pop(0)
                            webroverlist['yhist'][tempindx].pop(0)
                    if tempindx == rover_indx:
                        if not webroverlist['hflag']:
                            set_logbox(text="Position updated.", color="black")
                        set_roverstate(rover)
                    updatecmd = 0
                    doc.add_next_tick_callback(update_CDS)
                elif rover.wlist[0] == 'ACK':  # State change
                    webroverlist['state'][tempindx] = rover.wlist[1]
                    if tempindx == rover_indx:
                        set_roverstate(rover)
                elif rover.wlist[0] == 'HEART':  # Heartbeat signal sent
                    webroverlist['hflag'][tempindx] = True
                    updatecmd = 0
                    doc.add_next_tick_callback(update_CDS)
                elif rover.wlist[0] == 'SILENT':  # Silent signal sent
                    webroverlist['hflag'][tempindx] = False
                    updatecmd = 0
                    doc.add_next_tick_callback(update_CDS)
                elif rover.wlist[0] == 'DOBS':  # Obstacle distance
                    pass  # To be implemented (add to map?)
                elif rover.wlist[0] == 'COL':
                    pass  # To be implemented (add to map?)
            rover.lock.release()

    # When selection in rovermenu changes
    def rovermenu_handler(attr, old, new):
        global rover_indx, webroverlist, seqdict, updatecmd
        try:
            rover_indx = webroverlist['name'].index(new)
            HEARTbtn.active = 0 if webroverlist['hflag'][rover_indx] else 1
            mainlock.acquire()
            set_roverstate(list_of_rovers[rover_indx])
            mainlock.release()
            seqdict['lines'] = webroverlist['seqlist'][rover_indx]
            seqfilein.value = webroverlist['seqfile'][rover_indx]
            updatecmd = 10
            doc.add_next_tick_callback(update_CDS)
        except ValueError:
            rover_indx = -1
            roverstate.text = ''

    def taptoolcallback(event):
        global rover_indx, webroverlist, updatecmd, eventx, eventy
        global gotodict, setposdict, turndict
        if len(data_s.selected.indices) == 1:
            rover_indx = data_s.selected.indices[0]
            rovermenu.value = rovermenu.options[rover_indx]
        elif len(data_s.selected.indices) == 0:
            if dropdown.value == 'GOTO' and not EDITmapbox.active:
                # goto_glyph.line_alpha = 1.0
                [gotodict['values'][0], gotodict['values'][1]] = [
                    float('%.3f' % event.x), float('%.3f' % event.y)]
                [eventx, eventy] = [event.x, event.y]
                updatecmd = 2
                doc.add_next_tick_callback(update_CDS)
            elif dropdown.value == 'SETPOS' and not EDITmapbox.active:
                [gotodict['values'][0], gotodict['values'][1]] = [
                    float('%.3f' % event.x), float('%.3f' % event.y)]
                [setposdict['values'][0], setposdict['values'][1]] = [
                    float('%.3f' % event.x), float('%.3f' % event.y)]
                [eventx, eventy] = [event.x, event.y]
                updatecmd = 3
                doc.add_next_tick_callback(update_CDS)
            elif dropdown.value == 'TURN' and not EDITmapbox.active:
                [gotodict['values'][0], gotodict['values'][1]] = [
                    float('%.3f' % event.x), float('%.3f' % event.y)]
                if rover_indx > -1:
                    destang = compute_angle(
                        xi=webroverlist['x'][rover_indx],
                        yi=webroverlist['y'][rover_indx],
                        xf=event.x, yf=event.y)
                    turndict['values'][0] = float('%.1f' % destang)
                    [eventx, eventy] = [event.x, event.y]
                    updatecmd = 4
                    doc.add_next_tick_callback(update_CDS)

    def doubletaptoolcallback(event):
        global updatecmd, eventx, eventy
        global gotodict, setposdict, turndict
        if dropdown.value == 'GOTO' and not EDITmapbox.active:
            # goto_glyph.line_alpha = 1.0
            [gotodict['values'][0], gotodict['values'][1]] = [
                float('%.3f' % event.x), float('%.3f' % event.y)]
            [eventx, eventy] = [event.x, event.y]
            updatecmd = 5
            doc.add_next_tick_callback(update_CDS)
        elif dropdown.value == 'SETPOS' and not EDITmapbox.active:
            [gotodict['values'][0], gotodict['values'][1]] = [
                float('%.3f' % event.x), float('%.3f' % event.y)]
            # goto_glyph.line_alpha = 1.0
            [setposdict['values'][0], setposdict['values'][1]] = [
                float('%.3f' % event.x), float('%.3f' % event.y)]
            [eventx, eventy] = [event.x, event.y]
            updatecmd = 6
            doc.add_next_tick_callback(update_CDS)
        elif dropdown.value == 'TURN' and not EDITmapbox.active:
            [gotodict['values'][0], gotodict['values'][1]] = [
                float('%.3f' % event.x), float('%.3f' % event.y)]
            if rover_indx > -1:
                destang = compute_angle(
                    xi=webroverlist['x'][rover_indx],
                    yi=webroverlist['y'][rover_indx],
                    xf=event.x, yf=event.y)
                turndict['values'][0] = float('%.1f' % destang)
                [eventx, eventy] = [event.x, event.y]
                updatecmd = 7
                doc.add_next_tick_callback(update_CDS)

    def table_callback(attr, old, new):
        global updatecmd
        if dropdown.value == 'GOTO':
            updatecmd = 8
            doc.add_next_tick_callback(update_CDS)

    def send_nofcmd(cmdstring='HALT'):
        global rover_indx
        if rover_indx > -1 and rover_indx < len(list_of_rovers):
            rover = list_of_rovers[rover_indx]
            rover.lock.acquire()
            rover.command = '<'+cmdstring+'>'
            rover.cflag = True
            set_logbox(text='Sent &lt;%s&gt; to %s.' % (cmdstring,
                                                        rover.name))
            rover.lock.release()

    def send_fcmd(cmdstring='OBS', fields=['', '']):
        global rover_indx
        if rover_indx > -1:
            rover = list_of_rovers[rover_indx]
            rover.lock.acquire()
            rover.command = '<'+cmdstring
            for field in fields:
                rover.command += ',' + field
            rover.command += '>'
            sentcmd = 'Sent &lt;%s&gt; to %s.' % (rover.command[1:-1],
                                                  rover.name)
            set_logbox(text=sentcmd)
            rover.cflag = True
            rover.lock.release()
            if cmdstring == 'SETPOS':
                webroverlist['xhist'][rover_indx] = [0]
                webroverlist['yhist'][rover_indx] = [0]

    def btns_callback(mybtn):
        global updatecmd, seqdict, webroverlist
        if mybtn == HALTallbtn:
            mainlock.acquire()
            for rover in list_of_rovers:
                if rover.alive:
                    rover.lock.acquire()
                    rover.command = '<HALT>'
                    rover.cflag = True
                    rover.lock.release()
            set_logbox(text='Sent &lt;HALT&gt; to all rovers.')
            mainlock.release()
        elif mybtn == HEARTallbtn:
            mainlock.acquire()
            for rover in list_of_rovers:
                if rover.alive:
                    rover.lock.acquire()
                    rover.command = '<HEART,>'
                    rover.cflag = True
                    rover.lock.release()
            set_logbox(text='Sent &lt;HEART,&gt; to all rovers.')
            if rover_indx > -1:
                webroverlist['hflag'] = [True]*len(webroverlist['hflag'])
                updatecmd = 0
                doc.add_next_tick_callback(update_CDS)
                HEARTbtn.active = 0
            mainlock.release()
        elif mybtn == SILENTallbtn:
            mainlock.acquire()
            for rover in list_of_rovers:
                if rover.alive:
                    rover.lock.acquire()
                    rover.command = '<SILENT>'
                    rover.cflag = True
                    rover.lock.release()
            set_logbox(text='Sent &lt;SILENT&gt; to all rovers.')
            if rover_indx > -1:
                webroverlist['hflag'] = [False]*len(webroverlist['hflag'])
                updatecmd = 0
                doc.add_next_tick_callback(update_CDS)
                HEARTbtn.active = 1
            mainlock.release()
        elif mybtn == PAUSEallbtn:
            mainlock.acquire()
            for rover in list_of_rovers:
                if rover.alive:
                    rover.lock.acquire()
                    rover.pause = True
                    rover.lock.release()
            mainlock.release()
            set_logbox(text="Pause flags set for all live rovers.")
        elif mybtn == UNPAUSEallbtn:
            mainlock.acquire()
            for rover in list_of_rovers:
                if rover.alive:
                    rover.lock.acquire()
                    rover.pause = False
                    rover.lock.release()
            mainlock.release()
            set_logbox(text="Pause flags unset for all live rovers.")
        elif mybtn == PAUSEbtn:
            if rover_indx > -1:
                mainlock.acquire()
                rover = list_of_rovers[rover_indx]
                rover.lock.acquire()
                rover.pause = True
                rover.lock.release()
                mainlock.release()
                set_logbox(text="Pause flag set on %s." %
                           webroverlist['name'][rover_indx])
        elif mybtn == PAUSEbtn:
            if rover_indx > -1:
                mainlock.acquire()
                rover = list_of_rovers[rover_indx]
                rover.lock.acquire()
                rover.pause = True
                rover.lock.release()
                mainlock.release()
                set_logbox(text="Pause flag set on %s." %
                           webroverlist['name'][rover_indx])
        elif mybtn == UNPAUSEbtn:
            if rover_indx > -1:
                mainlock.acquire()
                rover = list_of_rovers[rover_indx]
                rover.lock.acquire()
                rover.pause = False
                rover.lock.release()
                mainlock.release()
                set_logbox(text="Pause flag unset on %s." %
                           webroverlist['name'][rover_indx])
        elif mybtn == RUNseqbtn:
            if rover_indx > -1 and len(webroverlist[
                    'seqlist'][rover_indx]) > 2:
                try:
                    _line1j = json.loads(webroverlist['seqlist'][
                        rover_indx][0])
                    webroverlist['loop'][rover_indx] = _line1j['start']
                    webroverlist['lflag'][rover_indx] = _line1j['loopflag']
                except (TypeError, KeyError, json.decoder.JSONDecodeError):
                    set_logbox(text="Bad json string in first row!",
                               color='red')
                else:
                    rover = list_of_rovers[rover_indx]
                    rover.lock.acquire()
                    rover.command = '<HALT>'
                    rover.clean_send()
                    rover.seqfile = 'sequences/'+webroverlist[
                        'seqfile'][rover_indx]
                    rover.seqlist = webroverlist['seqlist'][rover_indx][1:]
                    rover.numseq = len(rover.seqlist)
                    rover.loopflag = _line1j['loopflag']
                    if _line1j['start'] < rover.numseq:
                        rover.superstate = _line1j['start']
                    else:
                        rover.superstate = rover.numseq - 1
                    rover.ackflag = False
                    rover.state = 0  # IDLE
                    rover.pause = False
                    rover.lock.release()
                    set_logbox(text="Sent sequence to %s" %
                               webroverlist['name'][rover_indx])
        elif mybtn == READseqbtn:
            if rover_indx > -1 and check_sane_str(seqfilein.value):
                _seqfile = 'sequences/'+seqfilein.value
                if os.path.exists(_seqfile):
                    set_logbox(text='Loading sequence: %s' %
                               _seqfile)
                    _fp = open(_seqfile, 'r')
                    _line1 = _fp.readline().rstrip('\n')
                    try:
                        _line1j = json.loads(_line1.rstrip())
                        [_line1j['loopflag'], _line1j['start']]
                    except (TypeError, KeyError, json.decoder.JSONDecodeError):
                        set_logbox(text='Invalid sequence file: %s' %
                                   seqfilein)
                        _fp.close()
                        seqdict['lines'] = []
                    else:
                        webroverlist['loop'][rover_indx] = _line1j['start']
                        webroverlist['lflag'][rover_indx] = _line1j['loopflag']
                        webroverlist['seqfile'][rover_indx] = seqfilein.value
                        webroverlist['seqlist'][rover_indx] = [_line1]
                        webroverlist['seqlist'][rover_indx].extend([
                            line.rstrip('\n').split(
                                '>', 1)[0] for line in _fp])
                        _fp.close()
                        # Remove empty strings
                        webroverlist['seqlist'][rover_indx][:] = [
                            x+'>' for x in webroverlist[
                                'seqlist'][rover_indx] if x != '']
                        webroverlist[
                            'seqlist'][rover_indx][0] = _line1.rstrip('>')
                        seqdict['lines'] = webroverlist[
                            'seqlist'][rover_indx]
                    updatecmd = 0
                    doc.add_next_tick_callback(update_CDS)
                else:
                    set_logbox("Sequence file %s not found." %
                               seqfilein.value)
            else:
                set_logbox(text="Bad filename or no rovers in list!",
                           color='red')
        elif mybtn == SAVEseqbtn:
            if rover_indx > -1 and check_sane_str(seqfilein.value):
                _seqfile = 'sequences/'+seqfilein.value
                webroverlist['seqlist'][rover_indx] = dict(
                    data_seq.data)['lines']
                _fp = open(_seqfile, 'w')
                for line in webroverlist['seqlist'][rover_indx]:
                    _fp.write(line+'\n')
                _fp.close()
                webroverlist['seqfile'][rover_indx] = seqfilein.value
                set_logbox(text="Sequence saved to %s." % _seqfile)
            else:
                set_logbox(text="No {'*', '/', '..'} or blank string allowed!",
                           color='red')
        elif mybtn == ADDROWseqbtn:
            if rover_indx > -1:
                webroverlist['seqlist'][rover_indx].append('')
                seqdict['lines'] = webroverlist['seqlist'][rover_indx]
                updatecmd = 10
                doc.add_next_tick_callback(update_CDS)
        elif mybtn == IDbtn:
            send_nofcmd('ID')
        elif mybtn == POSbtn:
            send_nofcmd('POS')
        elif mybtn == HALTbtn:
            send_nofcmd('HALT')
        elif mybtn == DIEbtn:
            send_nofcmd('DIE')
        elif mybtn == SENDbtn:
            fields = [str(i) if i != None else ''
                      for i in dict(fcmdsource.data)['values']]
            send_fcmd(cmdstring=dropdown.value, fields=fields)

    def HEARTbtn_handler(attr, old, new):
        global rover_indx, webroverlist, data_s, updatecmd
        if new == 0:
            send_nofcmd('HEART,')
            webroverlist['hflag'][rover_indx] = True
        else:
            send_nofcmd('SILENT')
            webroverlist['hflag'][rover_indx] = False
        updatecmd = 0
        doc.add_next_tick_callback(update_CDS)

    def oldstore_fcmd(old):
        global setposdict, fwddict, bwddict, cfwddict, cbwddict, turndict
        global aturndict, cturndict, gotodict, obsdict, pobsdict, searchdict
        global fcmdsource
        if old == 'SETPOS':
            setposdict = dict(fcmdsource.data)
        elif old == 'FWD':
            fwddict = dict(fcmdsource.data)
        elif old == 'BWD':
            bwddict = dict(fcmdsource.data)
        elif old == 'CFWD':
            cfwddict = dict(fcmdsource.data)
        elif old == 'CBWD':
            cbwddict = dict(fcmdsource.data)
        elif old == 'TURN':
            turndict = dict(fcmdsource.data)
        elif old == 'ATURN':
            aturndict = dict(fcmdsource.data)
        elif old == 'CTURN':
            cturndict = dict(fcmdsource.data)
        elif old == 'GOTO':
            gotodict = dict(fcmdsource.data)
            # goto_glyph.line_alpha = 0.0
        elif old == 'OBS':
            obsdict = dict(fcmdsource.data)
        elif old == 'POBS':
            pobsdict = dict(fcmdsource.data)
        elif old == 'SEARCH':
            searchdict = dict(fcmdsource.data)

    def dropdown_handler(attr, old, new):
        global setposdict, fwddict, bwddict, cfwddict, cbwddict, turndict
        global aturndict, cturndict, gotodict, obsdict, pobsdict, searchdict
        global fcmdsource, tempglobdict, updatecmd
        oldstore_fcmd(old)
        if new == 'SETPOS':
            tabletitle.text = "Set Position"
            tempglobdict = setposdict
        elif new == 'FWD':
            tabletitle.text = "Move forward by distance"
            tempglobdict = fwddict
        elif new == 'BWD':
            tabletitle.text = "Move backward by distance"
            tempglobdict = bwddict
        elif new == 'CFWD':
            tabletitle.text = "Continuous move forward"
            tempglobdict = cfwddict
        elif new == 'CBWD':
            tabletitle.text = "Continous move backward"
            tempglobdict = cbwddict
        elif new == 'TURN':
            tabletitle.text = "Turn to angle (with x-axis)"
            tempglobdict = turndict
        elif new == 'ATURN':
            tabletitle.text = "Turn by angle (0: CCW, 1: CW)"
            tempglobdict = aturndict
        elif new == 'CTURN':
            tabletitle.text = "Continuous turn (0: CCW, 1: CW)"
            tempglobdict = cturndict
        elif new == 'GOTO':
            tabletitle.text = "Go to (x, y) and turn to end angle"
            tempglobdict = gotodict
        elif new == 'OBS':
            tabletitle.text = "Report distance of obstacle at angle"
            tempglobdict = obsdict
        elif new == 'POBS':
            tabletitle.text = "Move distance away from obstacle"
            tempglobdict = pobsdict
        elif new == 'SEARCH':
            tabletitle.text = "Search for RFID tag within radius"
            tempglobdict = searchdict
        updatecmd = 9
        doc.add_next_tick_callback(update_CDS)

    def showhide_callback(attr, old, new):
        hist_lines.visible = 0 in new
        mapvisible = 1 in new
        obstacles.visible = mapvisible
        walls.visible = mapvisible
        rfidtags.visible = mapvisible

    def editmap_callback(attr, old, new):
        mapeditable = 0 in new
        fctxt.editable = mapeditable
        obs_edit_tool.add = mapeditable
        obs_edit_tool.drag = mapeditable
        rfid_edit_tool.add = mapeditable
        rfid_edit_tool.drag = mapeditable
        rfidtable.editable = mapeditable

    def LOADmapbtn_handler():
        load_mapfile(mapfilein.value)

    def hidewarntab():
        warntabs.visible = False
        showhide.disabled = False
        mapfilein.disabled = False
        LOADmapbtn.disabled = False
        SAVEmapbtn.disabled = False

    def YESbtn_handler():
        mapread = [dict(dnorth.data), dict(dobs.data),
                   dict(dwalls.data), dict(drfid.data)]
        mapfile = 'maps/' + mapfilein.value
        fp = open(mapfile, 'w')
        json.dump(mapread, fp)
        fp.close()
        set_logbox(text="Map saved to %s (overwritten)." % mapfile)
        hidewarntab()

    def CANCELbtn_handler():
        hidewarntab()

    btnlist = [HALTallbtn, HEARTallbtn, SILENTallbtn,
               PAUSEallbtn, UNPAUSEallbtn, PAUSEbtn, UNPAUSEbtn,
               READseqbtn, SAVEseqbtn, RUNseqbtn, ADDROWseqbtn,
               IDbtn, POSbtn, HALTbtn, DIEbtn, SENDbtn]

    # Callbacks
    for btn in btnlist:
        btn.on_click(partial(btns_callback, mybtn=btn))
    HEARTbtn.on_change('active', HEARTbtn_handler)
    dropdown.on_change("value", dropdown_handler)
    mapp.on_event(Tap, taptoolcallback)
    mapp.on_event(DoubleTap, doubletaptoolcallback)
    fcmdsource.on_change('data', table_callback)
    dnorth.on_change('data', fcsl_callback)  # Compass angle change
    doc.add_periodic_callback(update, 500)  # Periodic callback
    rovermenu.on_change("value", rovermenu_handler)  # When selection changes
    showhide.on_change('active', showhide_callback)
    EDITmapbox.on_change('active', editmap_callback)
    LOADmapbtn.on_click(LOADmapbtn_handler)
    SAVEmapbtn.on_click(SAVEmapbtn_handler)
    YESbtn.on_click(YESbtn_handler)
    CANCELbtn.on_click(CANCELbtn_handler)

    leftpanel = column([HALTallbtn, row([HEARTallbtn, SILENTallbtn]),
                        row([PAUSEallbtn, UNPAUSEallbtn]),
                        row([column([DIEbtn, Spacer(height=10),
                                     rovermenu]),
                             roverstate]), Spacer(height=30),
                        nofieldbtngrid, HEARTbtn, Spacer(height=30),
                        rovtabs])
    centerpanel = column([mapp, logbox])
    rightpanel = column([row([fc, fctxt]), showhide, maptabs, warntabs])
    frontend = row([leftpanel, centerpanel, rightpanel])
    doc.add_root(frontend)


def webUIfunc():
    # Start this in its own kill-able thread
    staticpath = 'web_templates'
    webUIserver = Server({'/webapp': webapp}, num_procs=1,
                         extra_patterns=[('/', IndexHandler),
                                         ('/web_templates/(.*)',
                                          StaticFileHandler,
                                          {'path': staticpath})])
    print('webUI: http://localhost:5006/')
    webUIserver.start()
    webUIserver.io_loop.instance().start()


# I'll need this to "gracefully" exit bokeh/tornado webserver thread
class thread_with_trace(threading.Thread):
    def __init__(self, *args, **keywords):
        threading.Thread.__init__(self, *args, **keywords)
        self.killed = False

    def start(self):
        self.__run_backup = self.run
        self.run = self.__run
        threading.Thread.start(self)

    def __run(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        sys.settrace(self.globaltrace)
        self.__run_backup()
        self.run = self.__run_backup

    def globaltrace(self, frame, event, arg):
        if event == 'call':
            return self.localtrace
        else:
            return None

    def localtrace(self, frame, event, arg):
        if self.killed:
            if event == 'line':
                raise SystemExit()
        return self.localtrace

    def kill(self):
        self.killed = True


roverlistchanged = False
roverchangetype = ''
list_of_rovers = []
streamhandler = []
filehandler = []
numerical_level = logging.WARNING
weblog = []
IP_address = ''  # Use 'localhost' for local testing only
Port = 8081

if __name__ == '__main__':

    from bokeh.util import logconfig
    logconfig.basicConfig(level=logging.WARNING)

    webUIflag = False
    try:
        opts, args = getopt.getopt(sys.argv[1:], "daipw",
                                   ["debug", "info",
                                    "address", "port", "with-webUI"])
    except getopt.GetoptError:
        print('usage: python ', sys.argv[0],
              '[-d|--debug|-i|--info] [-a|--address <IP_address>] '
              '[-p|--port <port_number>] [-w|--with-webUI]')
        sys.exit(0)
    numerical_level = logging.WARNING
    logfmt = '%(name)s:%(levelname)s:\t%(message)s'
    logfmt = '%(asctime)s:'+logfmt
    for opt, arg in opts:
        if opt in ["-i", "--info"]:
            numerical_level = logging.INFO
        elif opt in ["-d", "--debug"]:
            numerical_level = logging.DEBUG
        elif opt in ["-a", "--address"]:
            IP_address = str(arg)
        elif opt in ["-p", "--port"]:
            Port = int(arg)
        elif opt in ["-w", "--with-webUI"]:
            # Start webUI stuff
            webUIthread = thread_with_trace(target=webUIfunc)
            webUIthread.start()
            webUIflag = True

    # Remove old formatting from all handlers in root logger
    root_logger = logging.getLogger()
    for h in root_logger.handlers:
        h.setFormatter(
            logging.Formatter('%(name)s:%(levelname)s:\t%(message)s'))
        h.setLevel(logging.WARNING)
    
    threadname = threading.currentThread().getName()
    mainlog = logging.getLogger('server:%s' % threadname)

    mainlog.setLevel(numerical_level)
    streamhandler = logging.StreamHandler(sys.stdout)
    streamhandler.setLevel(numerical_level)
    streamformatter = logging.Formatter('%(name)s:%(levelname)s:\t%(message)s')
    streamhandler.setFormatter(streamformatter)

    filehandler = logging.FileHandler(logfilename)
    filehandler.setLevel(numerical_level)
    fileformatter = logging.Formatter(logfmt, datefmt='%I:%M:%S %p')
    filehandler.setFormatter(fileformatter)

    mainlog.addHandler(filehandler)
    mainlog.addHandler(streamhandler)

    # weblog = logging.getLogger('webUI:')
    # weblog.setLevel(numerical_level)
    # weblog.addHandler(filehandler)
    # weblog.addHandler(streamhandler)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server.bind((IP_address, Port))
    server.listen(10)

    soclist = [server, sys.stdin]
    running = True
    inbuffer = ''

    print('Running')
    while running:
        # 1. Check for new connections
        # 2. Kill dead rovers and remove from list_of_rovers
        # 3. Respond to field-rover messages that were passed upstream
        # 4. Handle user stdin input

        inlist, outlist, exlist = select.select(soclist, [], [], 0.1)
        # 1. Handle new rovers joining
        if server in inlist:
            conn, addr = server.accept()
            newrover = Rover(conn, addr,
                             streamhandler, filehandler, numerical_level)
            mainlock.acquire()
            list_of_rovers.append(newrover)
            roverlistchanged = True
            roverchangetype = 'NEW'
            mainlock.release()
            mainlog.info('<'+addr[0]+'>'+" connected")

        # 2. Prune list_of_rovers[] for dead connections
        for rover in list_of_rovers:
            if not rover.alive:
                rover.thread.join()
                mainlog.debug('removed dead rover <'+rover.addr[0]+'>')
                mainlock.acquire()
                roverindex = list_of_rovers.index(rover)
                list_of_rovers.remove(rover)
                roverlistchanged = True
                roverchangetype = 'DIE:%d' % roverindex
                mainlock.release()

        # 3. Respond to flagged field-rover messages passed upstream
        mainlock.acquire()
        for rover in list_of_rovers:
            rover.lock.acquire()
            if rover.alive and rover.sflag:
                if rover.smsg_buffer:
                    upmsg = rover.smsg_buffer.pop(0)
                    if upmsg[0:5] == '<MYID':
                        mainlog.debug("ID change detected.")
                    elif upmsg[0:5] == '<SYN,':
                        # Broadcast string to all other rovers
                        mainlog.info(
                            "Broadcasting SYN string '%s' from %s." %
                            (upmsg[5:-1], rover.name))
                        b_str = upmsg[5:-1]
                        for rover2 in list_of_rovers:
                            if rover2 != rover:
                                rover2.lock.acquire()
                                rover2.syn_str.append(b_str)
                                rover2.lock.release()
                    else:
                        temptxt = "Code for %s from %s pending."
                        mainlog.debug(temptxt % (upmsg, rover.name))
                if len(rover.smsg_buffer) == 0:
                    rover.sflag = False
            rover.lock.release()
        mainlock.release()

        # 4. Handle user stdin input
        if sys.stdin in inlist:
            userinput = sys.stdin.readline()
            if userinput != '\n':
                userinput = userinput.split()[0]
            if userinput == 'quit':
                running = False
                if webUIflag:
                    webUIthread.kill()
                    webUIthread.join()
                for rover in list_of_rovers:
                    rover.die()
                    rover.thread.join()
            elif userinput == 'names':
                mainlock.acquire()
                for rover in list_of_rovers:
                    if rover.alive:
                        print([rover.name, rover.version,
                               rover.thread.getName()])
                mainlock.release()
            elif userinput[0:4] == 'CMD:':
                fields = userinput.split(':')
                if len(fields) == 4:
                    for rover in list_of_rovers:
                        if [fields[1], fields[2]] == [
                                rover.name, rover.thread.getName()]:
                            rover.lock.acquire()
                            rover.command = fields[3]
                            rover.cflag = True
                            rover.lock.release()
                            break
                else:
                    print("Malformed input: %s" % userinput)
            elif userinput[0:4] == 'SEQ:':
                fields = userinput.split(':')
                if len(fields) == 4:
                    for rover in list_of_rovers:
                        if [fields[1], fields[2]] == [
                                rover.name, rover.thread.getName()]:
                            rover.lock.acquire()
                            rover.seqfile = fields[3]
                            rover.superstate = -2
                            rover.lock.release()
                            break
                else:
                    print("Malformed input: %s" % userinput)
            elif userinput[0] == '<':
                # broadcast to all live rovers
                for rover in list_of_rovers:
                    if rover.alive:
                        rover.lock.acquire()
                        rover.command = userinput
                        rover.cflag = True
                        rover.lock.release()
            else:
                print("Unrecognized input: %s" % userinput)

    try:
        conn.close()
    except NameError:
        pass
    server.close()
    # Close logging handlers
    for handler in mainlog.handlers:
        handler.close()
        mainlog.removeFilter(handler)
