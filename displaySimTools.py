#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, Slider, TextBox
import matplotlib.image as mpimg
import matplotlib as mpl

class displayTools:

    def __init__(self, config):
        self.config = config #allows for copies of the base config to be made with variations
        self.moveCount = 0

    def setUpPlot(self, adjustBottom = ''):
        """set up the axes, window, and size of the plot"""

        self.figure, self.axes = plt.subplots()

        if self.config.enableClicking:
            self.figure.canvas.mpl_connect('button_press_event', self.clickGraph)

        if self.config.enableMoving:
            self.figure.canvas.mpl_connect('motion_notify_event', self.moveGraph)

        #check if resized graph: https://matplotlib.org/devdocs/users/event_handling.html
        if self.config.enableResizing:
            self.figure.canvas.mpl_connect('resize_event', self.userResized)

        #plt.close('all')#kind of fixes animation issue, but not really since closes and re-open plot every time updates
        #https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.subplots_adjust.html#matplotlib.pyplot.subplots_adjust
        #adjust top and bottom of plot
        if adjustBottom != '':
            plt.subplots_adjust(bottom=adjustBottom)
        
        self.axes.cla() #clears the plot

        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

    def moveGraph(self, event):
        mapX = event.xdata
        mapY = event.ydata
        displayX = event.x
        displayY = event.y

        self.moveCount += 1

        if self.moveCount >= self.config.moveCountMax:
            self.moveCount = 0
            self.config.moveFunction(mapX, mapY, displayX, displayY)

    def clickGraph(self, event):
        button = int(event.button)
        mapX = event.xdata
        mapY = event.ydata
        displayX = event.x
        displayY = event.y

        self.config.clickFunction(button, mapX, mapY, displayX, displayY)

    def userResized(self, event):
        """reset the boundaries for what is and is not clickable"""
        xWidthSmall, xWidthBig, yWidthSmall, yWidthBig = self.figSize()
        #print('graph space:', self.config.graphSpace)
        self.config.updateClickArea(xWidthSmall, xWidthBig, yWidthSmall, yWidthBig)

    def loadGUI(self, buttons, sliders, textBoxes):
        """loads the GUI elements"""

        #add buttons
        self.buttons = {}
        for button in buttons:
            self.addButton(buttons[button])

        #add sliders
        self.sliders = {}
        for slider in sliders:
            self.addSlider(sliders[slider])

        #add textBoxes
        self.textBoxes = {}
        for textBox in textBoxes:
            self.addTextBox(textBoxes[textBox])

    def loadWindow(self, windowSize, legend, title, grid = True):
        """loads the window with a empty plot, do this when updating plot from GUI event/action"""

        #removes plotted elements from plot
        self.axes.cla()
        #removes everything from plot, not good
        #plt.clf()

        #grid on plot
        self.axes.grid(True)

        #set up window size: x, y, w, h
        #plt.axis("equal") #set equal scaling ############this is what breaks the slider, this is the bad way
        #https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.axes.Axes.set_adjustable.html#matplotlib.axes.Axes.set_adjustable
        plt.gca().set_aspect('equal', adjustable='datalim')

        xMin = windowSize[0] - windowSize[2]*.5
        xMax = windowSize[0] + windowSize[2]*.5
        yMin = windowSize[1] - windowSize[3]*.5
        yMax = windowSize[1] + windowSize[3]*.5

        #now set axes limits
        self.axes.set(xlim = (xMin, xMax), ylim = (yMin, yMax))

        #set axis titles
        #https://matplotlib.org/3.1.0/gallery/pyplots/fig_axes_labels_simple.html
        self.axes.set_xlabel(legend[0])
        self.axes.set_ylabel(legend[1])
        self.axes.set_title(title)

    def plotShapes(self, rectangleData, circleData, arrowData, rotRectangles = []):
        """plot the shapes on the graph that updates over time"""

        #plot rectangles
        for data in rectangleData:
            self.plotRectangle(data[0], data[1])

        #plot circles
        for data in circleData:
            self.plotCircle(data[0], data[1], data[2])
            #print(data)

        #plot arrows
        self.arrowCount = 0
        for data in arrowData:
            if len(data) == 4:
                self.plotArrow(data[0], data[1], data[2], data[3])
            else:
                self.plotArrow(data[0], data[1], data[2], data[3], data[4], data[5])
        if self.arrowCount > 0:
            print('One or more of arrow dx, dy were 0, set to [' + str(self.config.arrowXDefault) + ',' + str(self.config.arrowYDefault) + '] to graph')

        #plot rotated rectangles
        for data in rotRectangles:
            self.plotRotatedRectangles(data[0], data[1], data[2]) #TODO, need to fix since is broken

        #https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.pause.html
        #used to allow the active figure to update, thus, the continous animation
        if not self.config.hidePlot:
            plt.pause(0.001)


    def plotRotatedRectangles(self, rectangles, angle, type):
        """plots rotated rectangles"""
        """plot rectangles on the graph, (x, y, w, h) , types are goal, obstacle, or car, angle is in radians"""

        #https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.patches.Rectangle.html
        #https://matplotlib.org/3.1.1/api/transformations.html#matplotlib.transforms.Affine2D
        #https://stackoverflow.com/questions/4285103/matplotlib-rotating-a-patch

        transform = mpl.transforms.Affine2D().rotate(angle) + self.axes.transData
        colorFromType, fillFromType = self.getColor(type)

        for rectangle in rectangles:
            r1 = patches.Rectangle((rectangle[0] - rectangle[2]/2, rectangle[1] - rectangle[3]/2), rectangle[2], rectangle[3], color=colorFromType, fill = fillFromType)
            r1.set_transform(transform)
            self.axes.add_patch(r1)

    def plotDots(self, x, y, color):
        """plots x, y coordinates with a color (direct matplotlib arg)"""
        self.axes.plot(x, y, color)

    def closePlot(self):
        """closes the plot"""
        plt.close()

    def plotImage(self, image):
        """plots an image"""
        if image == '':
            #no image given
            return 0
        img = mpimg.imread(image)
        imgplot = plt.imshow(img)

    def showPlot(self):
        """shows the current plot"""
        plt.show()

    def figSize(self):
        """returns the figures bounding values, xmin, xmax, ymin, ymax"""
        #window = [self.figure.get_figwidth()*self.figure.get_dpi(), self.figure.get_figheight()*self.figure.get_dpi()]
        bbox = self.axes.get_window_extent()
        return bbox.x0, bbox.x1, bbox.y0, bbox.y1

    def addButton(self, buttonData):
        """add a button to the plot"""
        #https://matplotlib.org/3.1.1/gallery/widgets/buttons.html
        #format for button data: location [bottom left x, bottom left y, w, h,], name string, function

        #where button is
        axButton = plt.axes(buttonData['location'])

        #setting up button
        self.buttons[buttonData['name']] = Button(axButton, buttonData['name'])

        #link to button event function
        self.buttons[buttonData['name']].on_clicked(buttonData['function'])

    def addSlider(self, sliderData):
        """add a slider to the plot"""
        #https://matplotlib.org/3.1.1/gallery/widgets/slider_demo.html
        #format for slider data: location [bottom left x, bottom left y, w, h,], 
        #name string, min value, max value, function, initialValue, stepValue if stepValue is -1 then ignore
        
        #where slider is
        axSlider = plt.axes(sliderData['location'], facecolor = sliderData['color'])

        #setting up slider
        if sliderData['stepValue'] == -1:
            self.sliders[sliderData['name']] = Slider(axSlider, sliderData['name'], sliderData['minValue'], sliderData['maxValue'], 
                            sliderData['initialValue'])
        else:
            self.sliders[sliderData['name']] = Slider(axSlider, sliderData['name'], sliderData['minValue'], sliderData['maxValue'], 
                            valinit = sliderData['initialValue'], valstep = sliderData['stepValue'])

        #link to slider update function
        self.sliders[sliderData['name']].on_changed(sliderData['function'])

    def addTextBox(self, textBoxData):
        """add a text box to the plot"""
        #https://matplotlib.org/stable/gallery/widgets/textbox.html
        #format for adding a text box: location [bottom left x, bottom left y, w, h,], name string, initial text, function

        #where textBox is
        axTextBox = self.figure.add_axes(textBoxData['location'])

        #setting up textBox
        self.textBoxes[textBoxData['name']] = TextBox(axTextBox, textBoxData['name'], initial = textBoxData['initialText'])

        #link to textBox upate function
        self.textBoxes[textBoxData['name']].on_submit(textBoxData['function'])

    def plotRectangle(self, rectangles, type):
        """plot rectangles on the graph, (x, y, w, h) , types are goal, obstacle, or car"""

        #https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.patches.Rectangle.html

        #set color and fill
        colorFromType, fillFromType = self.getColor(type)
        for rectangle in rectangles:
            self.axes.add_patch(patches.Rectangle((rectangle[0] - rectangle[2]/2, rectangle[1] - rectangle[3]/2), 
                                                    rectangle[2], rectangle[3], color=colorFromType, fill = fillFromType))

    def plotCircle(self, circles, radius, type):
        """plot a circle on the graph"""

        #https://matplotlib.org/3.2.1/api/_as_gen/matplotlib.patches.Circle.html
        colorFromType, fillFromType = self.getColor(type)
        for circle in circles:
            self.axes.add_patch(patches.Circle(circle, radius, color = colorFromType, fill = fillFromType))

    def plotArrow(self, xStart, yStart, dx, dy, width = .1, head = .8):
        """plot an arrow on the graph"""
        if dx == dy and dx == 0:
            self.arrowCount += 1
            dx = self.config.arrowXDefault
            dy = self.config.arrowYDefault
        #https://matplotlib.org/3.2.1/gallery/text_labels_and_annotations/arrow_simple_demo.html
        self.axes.arrow(xStart, yStart, dx, dy, width = width, length_includes_head = True, head_width = head)

    def getColor(self, data):
        """return the color data and fill type, data = [color:str, fill :bool]"""

        #can convert this to a config as a dictionary or enum later
        if data[0] == "green":
            colorFromType = 'g' #green
        elif data[0] == "blue":
            colorFromType = 'b' #blue
        elif data[0] == "black":
            colorFromType = 'k' #black
        elif data[0] == "red":
            colorFromType = 'r' #red
        else:
            colorFromType = data[0]

        return colorFromType, data[1]

    def saveGraph(self, path):
        """saves a picture of the graph"""
        plt.savefig(path + '.png')
