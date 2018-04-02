/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2014 Open Ephys

    ------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "IMUReaderEditor.h"
#include <stdio.h>


IMUReaderEditor::IMUReaderEditor(GenericProcessor* parentNode, bool useDefaultParameterEditors=true)
    : GenericEditor(parentNode, useDefaultParameterEditors), lastFrameCount(0)

{
    desiredWidth = 220;

	// device label
	deviceLabel = new Label("devLabel", "Device");
	deviceLabel->setBounds(5,25,80,20);
    deviceLabel->setFont(Font("Small Text", 12, Font::plain));
    deviceLabel->setColour(Label::textColourId, Colours::darkgrey);
	addAndMakeVisible(deviceLabel);

	// device list
    deviceSelector = new ComboBox();
    deviceSelector->setBounds(70,25,140,20);
	deviceSelector->addListener(this);
    addAndMakeVisible(deviceSelector);

	// button to update device list
    updateButton = new UtilityButton("Update", Font ("Small Text", 12, Font::plain));
    updateButton->addListener(this);
    updateButton->setBounds(70, 50, 75, 20);
    addAndMakeVisible(updateButton);

	// status label
	statusLabel = new Label("statusLabel", "Status");
	statusLabel->setBounds(5,75,100,20);
    statusLabel->setFont(Font("Small Text", 12, Font::plain));
    statusLabel->setColour(Label::textColourId, Colours::darkgrey);
	addAndMakeVisible(statusLabel);

	// status text
	statusText = new Label("statusText", "not connected");
	statusText->setBounds(70,75,150,20);
    statusText->setFont(Font("Small Text", 10, Font::plain));
    statusText->setColour(Label::textColourId, Colours::darkgrey);
	addAndMakeVisible(statusText);

	// FPS label
	fpsLabel = new Label("fpsLabel", "FPS: 0");
	fpsLabel->setBounds(5,100,100,20);
    fpsLabel->setFont(Font("Small Text", 12, Font::plain));
    fpsLabel->setColour(Label::textColourId, Colours::darkgrey);
	addAndMakeVisible(fpsLabel);

	updateDeviceList();
	startTimer(1000);  // update FPS label once per second
}


IMUReaderEditor::~IMUReaderEditor()
{
}

void IMUReaderEditor::updateSettings()
{
	IMUReader* proc = (IMUReader*) getProcessor();
}


void IMUReaderEditor::updateDeviceList()
{
	IMUReader* proc = (IMUReader*) getProcessor();
	vector <ofSerialDeviceInfo> devices = serial.getDeviceList();
	String currentDevice = proc->getDevice();

	deviceSelector->clear(dontSendNotification);

    deviceSelector->addItem("None", 1);
	int listIndex = -1;
    for (int i = 0; i < devices.size(); i++)
    {
        deviceSelector->addItem(devices[i].getDevicePath(), i+2);
		if (currentDevice == devices[i].getDevicePath())
		{
			listIndex = i+2;
		}
    }
	if (listIndex > 0)
	{
		deviceSelector->setSelectedId(listIndex, dontSendNotification);
		if (proc->setDevice(currentDevice) == 0)
		{
			statusText->setText("connected", dontSendNotification);
		}
	}
	else
	{
    	deviceSelector->setSelectedId(1, dontSendNotification);
		proc->setDevice("");
		statusText->setText("not connected", dontSendNotification);
	}
}


void IMUReaderEditor::buttonEvent(Button* button)
{
	if (button == updateButton)
	{
		updateDeviceList();
	}
}


void  IMUReaderEditor::comboBoxChanged(ComboBox* combo)
{
	IMUReader* proc = (IMUReader*) getProcessor();

    if (combo == deviceSelector)
    {
        if (deviceSelector->getText() == "None")
        {
            proc->setDevice(String(""));
			statusText->setText("not connected", dontSendNotification);
        }
		else
        {
        	if (proc->setDevice(deviceSelector->getText()) == 0)
			{
				statusText->setText("connected", dontSendNotification);
			}
			else
			{
				statusText->setText("not connected", dontSendNotification);
			}
        }
    }
}


void IMUReaderEditor::disableControls()
{
	IMUReader* proc = (IMUReader*) getProcessor();

	updateButton->setEnabledState(false);
	deviceSelector->setEnabled(false);
}


void IMUReaderEditor::enableControls()
{
	IMUReader* proc = (IMUReader*) getProcessor();

	updateButton->setEnabledState(true);
	deviceSelector->setEnabled(true);
}

void IMUReaderEditor::timerCallback()
{
	IMUReader* proc = (IMUReader*) getProcessor();
	juce::int64 frameCount = proc->getFrameCount();
	juce::int64 fps = frameCount - lastFrameCount;
	lastFrameCount = frameCount;

	fpsLabel->setText(String("FPS: ") + String(fps), dontSendNotification);
}


