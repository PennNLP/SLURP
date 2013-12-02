"""
A web interface for Pragbot
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
import sys
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[web]")
    
class PragbotWeb(object):
    def __init__(self):
        self.urls = self._default_urls()  
        self.app = web.application(self.urls,globals())
        self.renderer = web.template.render('templates/')       

    def run(self):        
        self.app.run()
        
    def _default_urls(self):
        return ('/', 'PragbotWeb')
    
    def GET(self):
        #return "Hello World!"
        return self.renderer.index()
    
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