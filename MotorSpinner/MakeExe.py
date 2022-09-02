from distutils.core import setup
import py2exe

includes = ['tkinter','serial','math','struct']
setup(console=['SpinnerUI.py'],options={'py2exe':{'packages':includes}})
# setup(options={"py2exe"{"includes": includes}},console=['SpinnerUI.py'])