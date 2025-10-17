#! /usr/bin/env python
from nicegui import app, ui
from rosys.analysis.logging_page import LoggingPage


def startup() -> None:
    @ui.page('/')
    def home_page() -> None:
        ui.label('Hello to your new project: basekit!').classes('text-4xl absolute-center')


    logging_groups = ['basekit', 'rosys', 'nicegui']
    LoggingPage(logging_groups)


@app.get('/status')
def status() -> dict[str, str]:
    return {'status': 'ok'}


app.on_startup(startup)

ui.run(title='basekit')
