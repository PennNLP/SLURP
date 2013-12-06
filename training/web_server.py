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
    
OK = "OK"
NOT_OK = "NOT_OK"
web.config.session_parameters.update(cookie_name="current_exercise", cookie_domain="localhost",cookie_value="go_simple") 
class PragbotWeb(object):
    def __init__(self):
        self.urls = self._default_urls()
        web.config.session_parameters.update(cookie_name="current_exercise", cookie_domain="localhost")                
        #cookie = web.cookies(current_exercise="go_simple")
        self.trainer = None   
        self.current_exercise = None
        self.app = web.application(self.urls,globals())        
        self.get_text = form.Form(
                         form.Textbox("Command:"),
                         form.Button("Send")                         
                         )        
        self.renderer = web.template.render('templates/')
        self.head = str(self.renderer.command_response())
        self.tail = "</html>" 

        self.current_page = ""

    def get_trainer(self):
        if self.current_exercise:
            #If the current exercise is already known, don't check the cookie
            if self.trainer:
                if self.trainer.current_exercise == self.current_exercise: 
                        return
            self.trainer = Go(self.current_exercise)
            return        
        current_exercise = web.cookies().get("current_exercise")
        #Get the cookie, set the trainer
        if current_exercise:
            if self.trainer:
                if self.trainer.current_exercise == current_exercise: 
                    return
            self.trainer = Go(current_exercise)
        else:
            self.trainer = Go()
            
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
        self.get_trainer()
        form = self.get_user_text()        
        prompt = self.trainer.get_current_prompt()
        command_prompt = self.renderer.command_prompt(prompt)        
        command_form = self.renderer.command_page(form)
        page = self.head + str(command_prompt) + str(command_form) + self.tail
        self.current_page = ""
        return page
        #return self.renderer.index()
    
    def POST(self): 
        form = self.get_user_text() 
        if not self.trainer:
            self.get_trainer()
            
        if not form.validates(): 
            return self.renderer.command_page(form)
        else:
            user_input = form['Command:'].value
            training_response = self.trainer.run_current_exercise(user_input)
            if training_response.startswith(OK):
                #If response is ok, set the cookie, set self.current_exercise
                training_response = training_response.lstrip(OK)         
                next_exercise = self.trainer.get_next_exercise()        
                self.current_exercise = next_exercise           
                web.setcookie("current_exercise", next_exercise, 3600)                              
            elif training_response.startswith(NOT_OK):
                #Leave cookie alone
                training_response = training_response.lstrip(NOT_OK)
            self.head = str(self.renderer.command_response(training_response))
            #return self.current_page            
            return self.GET()
            
            #return "Received Command: %s and got response from training: %s" % (form['Command:'].value,self.current_page)
        
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