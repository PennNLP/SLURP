"""
A web interface for SLURP semantic command interpretation.
"""

# Copyright (C) 2013 Taylor Turpen
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
import web
from web import form
import sys
import logging
from training.basic_training import Go

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[web]")
    
class PragbotWeb(object):
    def __init__(self):
        self.urls = self._default_urls()  
        self.trainer = Go()
        self.app = web.application(self.urls,globals())        
        self.get_text = form.Form(
                         form.Textbox("Command:"),
                         form.Button("Send")                         
                         )
        self.renderer = web.template.render('templates/')       

    def run(self):    
        self.app.run()
        
    def get_user_text(self):
        return form.Form(form.Textbox("Command:"),
                         form.Button("Send"),
                         )
        
    def _default_urls(self):
        return ('/', 'PragbotWeb')
    
    def GET(self):
        #return "Hello World!"
        form = self.get_user_text()
        prompt = self.trainer.get_next_prompt()
        command_prompt = self.renderer.command_prompt(prompt)        
        command_form = self.renderer.command_page(form)
        return str(command_prompt) + str(command_form)
        #return self.renderer.index()
    
    def POST(self): 
        form = self.get_text() 
        if not form.validates(): 
            return self.renderer.command_page(form)
        else:
            # form.d.boe and form['boe'].value are equivalent ways of
            # extracting the validated arguments from the form.
            return "Received Command: %s" % (form['Command:'].value)
    
def main():
    args = ["/home/taylor/repos/Pragbot/build/jar/"]
    if len(args) < 1:
        logger.warning("Too few arguments, please include the path to the Pragbot jars.")
        return
    else:
        pragbot_web = PragbotWeb()
        pragbot_web.run()
        
    
if __name__ == '__main__':
    main()