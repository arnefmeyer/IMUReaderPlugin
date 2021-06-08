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

#ifndef __IMUREADEREDITOR_H__
#define __IMUREADEREDITOR_H__


#include <EditorHeaders.h>
#include "IMUReader.h"
#include <SerialLib.h>

/**

  User interface for the IMUReader plugin

  @see IMUReader

*/

class IMUReaderEditor : public GenericEditor, public ComboBox::Listener

{
public:
    IMUReaderEditor(GenericProcessor* parentNode, bool useDefaultParameterEditors);
    virtual ~IMUReaderEditor();

	void updateSettings();

	void buttonEvent(Button* button);
	void labelTextChanged(juce::Label *);
    void comboBoxChanged(ComboBox* combo);

    void disableControls();
	void enableControls();

	void timerCallback();

private:

	ScopedPointer<Label> deviceLabel;
	ScopedPointer<ComboBox> deviceSelector;
	ScopedPointer<UtilityButton> updateButton;
	ScopedPointer<Label> statusLabel;
	ScopedPointer<Label> statusText;
	ScopedPointer<Label> fpsLabel;

	juce::int64 lastFrameCount;

	ofSerial serial;

	void updateDeviceList();

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(IMUReaderEditor);

};


#endif  // __IMUREADEREDITOR_H__

