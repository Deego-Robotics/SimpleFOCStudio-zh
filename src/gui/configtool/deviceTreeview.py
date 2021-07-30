#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QTreeWidget
from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit
from src.simpleFOCConnector import SimpleFOCDevice
from src.simpleFOCConnector import Command

class DeviceTreeView(QTreeWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.device = SimpleFOCDevice.getInstance()

        self.sFOCDevice = QtWidgets.QTreeWidgetItem(self)
        self.sFOCDevice.setText(0, 'sFOC 设备')
        self.sFOCDevice.setIcon(0, GUIToolKit.getIconByName('motor'))

        self.setColumnCount(2)

        self.motionControl = QtWidgets.QTreeWidgetItem(self.sFOCDevice)
        self.motionControl.setText(0, '运动控制设置')
        self.motionControl.setIcon(0, GUIToolKit.getIconByName('pidconfig'))
        self.sFOCDevice.addChild(self.motionControl)
        
        self.controller = QtWidgets.QTreeWidgetItem(self.motionControl)
        self.controller.setText(0, '运动控制类型')
        self.controller.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.selectorControlLoop = QtWidgets.QComboBox(self)
        self.selectorControlLoop.addItems(['力矩控制', '速度控制', '角度控制', '速度开环控制', '角度开环控制'])
        self.selectorControlLoop.currentIndexChanged.connect(self.changeControlLoop)
        self.setItemWidget(self.controller,1,self.selectorControlLoop)

        self.torque = QtWidgets.QTreeWidgetItem(self.motionControl)
        self.torque.setText(0, '力矩控制类型')
        self.torque.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.selectorTorque = QtWidgets.QComboBox(self)
        self.selectorTorque.addItems(['电压', 'DC电流', 'FOC电流'])
        self.selectorTorque.currentIndexChanged.connect(self.changeTorque)
        self.setItemWidget(self.torque,1,self.selectorTorque)
        
        self.motionDownsample = QtWidgets.QTreeWidgetItem(self.motionControl)
        self.motionDownsample.setText(0, '运动控制频率降采样')
        self.motionDownsample.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.motionDownsample.setText(1, '')
        self.motionDownsample.setFlags(
            self.motionDownsample.flags() | QtCore.Qt.ItemIsEditable)
        
        self.PIDVelocityConfig = self.addPIDSubtree(self.motionControl,'速度 PID')
        self.PIDAngleConfig = self.addPIDSubtree(self.motionControl,'角度 PID')
        self.PIDCurrentQConfig = self.addPIDSubtree(self.motionControl,'电流q PID')
        self.PIDCurrentDConfig = self.addPIDSubtree(self.motionControl,'电流d PID')

        self.limitsConfig = QtWidgets.QTreeWidgetItem(self.sFOCDevice)
        self.limitsConfig.setText(0, '限幅')
        self.limitsConfig.setIcon(0, GUIToolKit.getIconByName('statistics'))
        self.sFOCDevice.addChild(self.limitsConfig)

        self.velocityLimit = QtWidgets.QTreeWidgetItem(self.limitsConfig)
        self.velocityLimit.setText(0, '速度限幅')
        self.velocityLimit.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.velocityLimit.setText(1, '')
        self.velocityLimit.setFlags(
            self.velocityLimit.flags() | QtCore.Qt.ItemIsEditable)

        self.voltageLimit = QtWidgets.QTreeWidgetItem(self.limitsConfig)
        self.voltageLimit.setText(0, '电压限幅')
        self.voltageLimit.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.voltageLimit.setText(1, '')
        self.voltageLimit.setFlags(
            self.voltageLimit.flags() | QtCore.Qt.ItemIsEditable)

        self.currentLimit = QtWidgets.QTreeWidgetItem(self.limitsConfig)
        self.currentLimit.setText(0, '电流限幅')
        self.currentLimit.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.currentLimit.setText(1, '')
        self.currentLimit.setFlags(
            self.currentLimit.flags() | QtCore.Qt.ItemIsEditable)

        self.statesConfig = QtWidgets.QTreeWidgetItem(self.sFOCDevice)
        self.statesConfig.setText(0, '状态')
        self.statesConfig.setIcon(0, GUIToolKit.getIconByName('statistics'))
        self.sFOCDevice.addChild(self.statesConfig)

        self.satateTarget = QtWidgets.QTreeWidgetItem(self.statesConfig)
        self.satateTarget.setText(0, '目标')
        self.satateTarget.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.satateTarget.setText(1, '')

        self.stateVq = QtWidgets.QTreeWidgetItem(self.statesConfig)
        self.stateVq.setText(0, '电压 q')
        self.stateVq.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.stateVd = QtWidgets.QTreeWidgetItem(self.statesConfig)
        self.stateVd.setText(0, '电压 d')
        self.stateVd.setIcon(0, GUIToolKit.getIconByName('gear'))

        self.stateCq = QtWidgets.QTreeWidgetItem(self.statesConfig)
        self.stateCq.setText(0, '电流 q')
        self.stateCq.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.stateCd = QtWidgets.QTreeWidgetItem(self.statesConfig)
        self.stateCd.setText(0, '电流 d')
        self.stateCd.setIcon(0, GUIToolKit.getIconByName('gear'))

        self.stateVel = QtWidgets.QTreeWidgetItem(self.statesConfig)
        self.stateVel.setText(0, '速度')
        self.stateVel.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.stateVel.setText(1, '')

        self.stateAngle = QtWidgets.QTreeWidgetItem(self.statesConfig)
        self.stateAngle.setText(0, '角度')
        self.stateAngle.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.stateAngle.setText(1, '')

        self.sensorConfig = QtWidgets.QTreeWidgetItem(self.sFOCDevice)
        self.sensorConfig.setText(0, '位置传感器设置')
        self.sensorConfig.setIcon(0, GUIToolKit.getIconByName('sensor'))
        self.sFOCDevice.addChild(self.sensorConfig)

        self.sensorZeroOffset = QtWidgets.QTreeWidgetItem(self.sensorConfig)
        self.sensorZeroOffset.setText(0, '零点偏置')
        self.sensorZeroOffset.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.sensorZeroOffset.setText(1, '')
        self.sensorZeroOffset.setFlags(
            self.sensorZeroOffset.flags() | QtCore.Qt.ItemIsEditable)

        self.sensorZeroElecOffset = QtWidgets.QTreeWidgetItem(self.sensorConfig)
        self.sensorZeroElecOffset.setText(0, '电气零点偏置')
        self.sensorZeroElecOffset.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.sensorZeroElecOffset.setText(1, '')
        self.sensorZeroElecOffset.setFlags(
            self.sensorZeroElecOffset.flags() | QtCore.Qt.ItemIsEditable)

        self.generalSettings = QtWidgets.QTreeWidgetItem(self.sFOCDevice)
        self.generalSettings.setText(0, '通用设置')
        self.generalSettings.setIcon(0, GUIToolKit.getIconByName('generalsettings'))
        self.sFOCDevice.addChild(self.generalSettings)

        self.phaseRes = QtWidgets.QTreeWidgetItem(self.generalSettings)
        self.phaseRes.setText(0, '相电阻')
        self.phaseRes.setIcon(0, GUIToolKit.getIconByName('res'))
        self.phaseRes.setText(1, '')
        self.phaseRes.setFlags(
            self.phaseRes.flags() | QtCore.Qt.ItemIsEditable)

        self.deviceStatus = QtWidgets.QTreeWidgetItem(self.generalSettings)
        self.deviceStatus.setText(0, '电机状态')
        self.deviceStatus.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.selectStatus = QtWidgets.QComboBox(self)
        self.selectStatus.addItems(['失能', '使能'])
        self.selectStatus.currentIndexChanged.connect(self.changeStatus)
        self.setItemWidget(self.deviceStatus,1,self.selectStatus)

        self.modulationType = QtWidgets.QTreeWidgetItem(self.generalSettings)
        self.modulationType.setText(0, 'PWM调制')
        self.modulationType.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.selectModulation = QtWidgets.QComboBox(self)
        self.selectModulation.addItems(['正弦PWM调制', '空间矢量PWM调制', '梯形调制 120', '梯形调制 150'])
        self.selectModulation.currentIndexChanged.connect(self.changeModType)
        self.setItemWidget(self.modulationType,1,self.selectModulation)

        self.modulationCenter = QtWidgets.QTreeWidgetItem(self.generalSettings)
        self.modulationCenter.setText(0, 'Modulation center')
        self.modulationCenter.setIcon(0, GUIToolKit.getIconByName('gear'))
        self.selectModCenter = QtWidgets.QComboBox(self)
        self.selectModCenter.addItems(['Disabled', 'Enabled'])
        self.selectModCenter.currentIndexChanged.connect(self.changeModCenter)
        self.setItemWidget(self.modulationCenter,1,self.selectModCenter)

        self.customComands = QtWidgets.QTreeWidgetItem(self.sFOCDevice)
        self.customComands.setText(0, 'Custom commands')
        self.customComands.setIcon(0, GUIToolKit.getIconByName('customcommands'))
        self.sFOCDevice.addChild(self.customComands)

        for customCommand in self.device.customCommands.customCommandsList:
            self.initCustomCommand(customCommand)

        self.installEventFilter(self)
        self.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.customCommandsMenu)

        self.header().resizeSection(0,230)

        self.setAlternatingRowColors(True)
        self.header().hide()
        self.expandItem(self.sFOCDevice)
        self.expandItem(self.motionControl)

        self.device.addConnectionStateListener(self)
        self.device.commProvider.commandDataReceived.connect(self.commandResponseReceived)
        self.device.commProvider.stateMonitorReceived.connect(self.stateResponseReceived)

        self.itemChanged.connect(self.treeItemEdited)

        self.setEnabled(self.device.isConnected)

    def customCommandsMenu(self, position):
        indexes = self.selectedIndexes()
        if len(indexes) > 0:
            level = 0
            index = indexes[0]
            while index.parent().isValid():
                index = index.parent()
                level += 1
        selectedItem = self.selectedItems()[0]
        menu = QtWidgets.QMenu()
        if selectedItem.text(0) == 'Custom commands':
            addComand = QtWidgets.QAction("Add command", self)
            addComand.triggered.connect(self.addCommandAction)
            menu.addAction(addComand)
        elif hasattr(selectedItem, 'isCustomCommand'):
            executeCommand = QtWidgets.QAction("Execute", self)
            executeCommand.triggered.connect(self.executeCustomCommandAction)
            menu.addAction(executeCommand)
            deleteCommand = QtWidgets.QAction("Remove", self)
            deleteCommand.triggered.connect(self.deleteCustomCommand)
            menu.addAction(deleteCommand)

        menu.exec_(self.viewport().mapToGlobal(position))

    def addCommandAction(self):
        selectedItem = self.selectedItems()[0]
        self.addCustomCommand(selectedItem)

    def executeCustomCommandAction(self):
        selectedItem = self.selectedItems()[0]
        selectedCustomCommand = selectedItem.text(1)
        self.device.sendCommand(selectedCustomCommand)

    def deleteCustomCommand(self):
        root = self.invisibleRootItem()
        deletedIndex = self.customComands.indexOfChild(self.selectedItems()[0])
        self.device.customCommands.customCommandsList.pop(deletedIndex)
        for item in self.selectedItems():
            (item.parent() or root).removeChild(item)

    def eventFilter(self, obj, event):
        if event.type() == QtCore.QEvent.KeyPress:
            if event.key() == QtCore.Qt.Key_Return:
                selectedItem = self.selectedItems()[0]
                if selectedItem.text(0) == 'Custom commands':
                    self.addCustomCommand(selectedItem)
            if event.key() == QtCore.Qt.Key_Space or event.key() == QtCore.Qt.Key_Right:
                selectedItem = self.selectedItems()[0]
                if selectedItem.parent().text(0) == 'Custom commands':
                    self.executeCustomCommandAction()
            if event.key() == QtCore.Qt.Key_Delete or event.key() == QtCore.Qt.Key_Backspace:
                selectedItem = self.selectedItems()[0]
                if selectedItem.parent().text(0) == 'Custom commands':
                    self.deleteCustomCommand()
        return super(DeviceTreeView, self).eventFilter(obj, event)

    def addCustomCommand(sefl,selectedItem):
        customCommand = QtWidgets.QTreeWidgetItem()
        customCommand.isCustomCommand = True
        customCommand.setText(0, '命令')
        customCommand.setIcon(0, GUIToolKit.getIconByName('gear'))

        customCommand.setFlags(
            customCommand.flags() | QtCore.Qt.ItemIsEditable)
        selectedItem.addChild(customCommand)
        sefl.device.customCommands.customCommandsList.append(Command('Command',''))

    def initCustomCommand(sefl, command):
        customCommand = QtWidgets.QTreeWidgetItem()
        customCommand.isCustomCommand = True
        customCommand.setText(0, command.cmdName)
        customCommand.setText(1, command.cmd)
        customCommand.setIcon(0, GUIToolKit.getIconByName('gear'))
        customCommand.setFlags(
            customCommand.flags() | QtCore.Qt.ItemIsEditable)
        sefl.customComands.addChild(customCommand)

    def addPIDSubtree(self, parent,  label):
        pidConfiguration = QtWidgets.QTreeWidgetItem()
        pidConfiguration.setText(0, label)
        pidConfiguration.setIcon(0, GUIToolKit.getIconByName('pidconfig'))
        parent.addChild(pidConfiguration)

        proportionalGain = QtWidgets.QTreeWidgetItem(pidConfiguration)
        proportionalGain.setText(0, 'P 比例项')
        proportionalGain.setIcon(0, GUIToolKit.getIconByName('gear'))
        proportionalGain.setText(1, '')
        proportionalGain.setFlags(
            proportionalGain.flags() | QtCore.Qt.ItemIsEditable)

        integralGain = QtWidgets.QTreeWidgetItem(pidConfiguration)
        integralGain.setText(0, 'I 积分项')
        integralGain.setIcon(0, GUIToolKit.getIconByName('gear'))
        integralGain.setText(1, '')
        integralGain.setFlags(
            integralGain.flags() | QtCore.Qt.ItemIsEditable)

        derivativeGain = QtWidgets.QTreeWidgetItem(pidConfiguration)
        derivativeGain.setText(0, 'D 微分项')
        derivativeGain.setIcon(0, GUIToolKit.getIconByName('gear'))
        derivativeGain.setText(1, '')
        derivativeGain.setFlags(
            derivativeGain.flags() | QtCore.Qt.ItemIsEditable)

        voltageRamp = QtWidgets.QTreeWidgetItem(pidConfiguration)
        voltageRamp.setText(0, '斜坡输出')
        voltageRamp.setIcon(0, GUIToolKit.getIconByName('gear'))
        voltageRamp.setText(1, '')
        voltageRamp.setFlags(
            voltageRamp.flags() | QtCore.Qt.ItemIsEditable)
        
        limit = QtWidgets.QTreeWidgetItem(pidConfiguration)
        limit.setText(0, '输出限幅')
        limit.setIcon(0, GUIToolKit.getIconByName('gear'))
        limit.setText(1, '')
        limit.setFlags(
            limit.flags() | QtCore.Qt.ItemIsEditable)

        lpfTf = QtWidgets.QTreeWidgetItem(pidConfiguration)
        lpfTf.setText(0, '低通滤波器')
        lpfTf.setIcon(0, GUIToolKit.getIconByName('gear'))
        lpfTf.setText(1, '')
        lpfTf.setFlags(
            lpfTf.flags() | QtCore.Qt.ItemIsEditable)

        return pidConfiguration

    def treeItemEdited(self, item, column):
        if item.parent().text(0) == 'Custom commands':
            updatedIndex = self.customComands.indexOfChild(item)
            updatedValue = item.text(column)
            if column == 0:
                self.device.customCommands.customCommandsList[
                    updatedIndex].cmdName = updatedValue
            else:
                self.device.customCommands.customCommandsList[
                    updatedIndex].cmd = updatedValue
        else:
            self.sendCommand(item, column)

    def sendCommand(self, item, column):
        value = item.text(1)
        fieldName = item.text(0)
        pidLabel = item.parent().text(0)
        pid = {}
        lpf = {}
        if '速度 PID' in pidLabel:
            pid = self.device.PIDVelocity
            lpf = self.device.LPFVelocity
        elif '角度 PID' in pidLabel:
            pid = self.device.PIDAngle
            lpf = self.device.LPFAngle
        elif '电流Q PID' in pidLabel:
            pid = self.device.PIDCurrentQ
            lpf = self.device.LPFCurrentQ
        elif '电流D PID' in pidLabel:
            pid = self.device.PIDCurrentD
            lpf = self.device.LPFCurrentD

        if 'P 比例项' in fieldName:
            self.device.sendProportionalGain(pid, value)
        elif 'I 积分项' in fieldName:
            self.device.sendIntegralGain(pid, value)
        elif 'D 微分项' in fieldName:
            self.device.sendDerivativeGain(pid, value)
        elif '斜坡输出' in fieldName:
            self.device.sendOutputRamp(pid, value)
        elif '低通滤波器' in fieldName:
            self.device.sendLowPassFilter(lpf, value)
        elif '输出限幅' in fieldName:
            self.device.sendOutputLimit(pid,value)
        elif '电压限幅' in fieldName:
            self.device.sendVoltageLimit(value)
        elif '速度限幅' in fieldName:
            self.device.sendVelocityLimit(value)
        elif '电流限幅' in fieldName:
            self.device.sendCurrentLimit(value)
        elif '相电阻' in fieldName:
            self.device.sendPhaseResistance(value)
        elif '零点偏置' in fieldName:
            self.device.sendSensorZeroOffset(value)
        elif '电气零点偏置' in fieldName:
            self.device.sendSensorZeroElectrical(value)
        elif '运动控制频率降采样' in fieldName:
            self.device.sendMotionDownsample(value)

    def refreshPIDSubtree(self, pidDisp, pidVal, lpfVal):
        pidDisp.child(0).setText(1,str(pidVal.P))
        pidDisp.child(1).setText(1,str(pidVal.I))
        pidDisp.child(2).setText(1,str(pidVal.D))
        pidDisp.child(3).setText(1,str(pidVal.outputRamp))
        pidDisp.child(4).setText(1,str(pidVal.outputLimit))
        pidDisp.child(5).setText(1,str(lpfVal.Tf))

    def commandResponseReceived(self, comandResponse):
        self.refreshDeviceTree()
        self.setTorqueMode(self.device.torqueType)
        self.setControlLopMode(self.device.controlType)
        self.setEnabledDisabled(self.device.deviceStatus)
        self.setModCenter(self.device.modulationCentered)
        self.setModType(self.device.modulationType)

    def stateResponseReceived(self, comandResponse):
        self.blockSignals(True)
        self.stateVel.setText(1,str(self.device.velocityNow))
        self.stateAngle.setText(1,str(self.device.angleNow))
        self.stateVd.setText(1,str(self.device.voltageDNow))
        self.stateVq.setText(1,str(self.device.voltageQNow))
        self.stateCq.setText(1,str(self.device.currentQNow))
        self.stateCd.setText(1,str(self.device.currentDNow))
        self.satateTarget.setText(1,str(self.device.targetNow))
        self.update()
        self.blockSignals(False)

    def refreshDeviceTree(self):
        self.blockSignals(True)

        self.refreshPIDSubtree( self.PIDVelocityConfig, self.device.PIDVelocity, self.device.LPFVelocity)
        self.refreshPIDSubtree( self.PIDAngleConfig, self.device.PIDAngle, self.device.LPFAngle)
        self.refreshPIDSubtree( self.PIDCurrentQConfig, self.device.PIDCurrentQ, self.device.LPFCurrentQ)
        self.refreshPIDSubtree( self.PIDCurrentDConfig, self.device.PIDCurrentD, self.device.LPFCurrentD)

        self.voltageLimit.setText(1,str(self.device.voltageLimit))
        self.velocityLimit.setText(1,str(self.device.velocityLimit))
        self.currentLimit.setText(1,str(self.device.currentLimit))

        self.sensorZeroOffset.setText(1,str(self.device.sensorZeroOffset))
        self.sensorZeroElecOffset.setText(1,str(self.device.sensorElectricalZero))
        
        self.phaseRes.setText(1,str(self.device.phaseResistance))
        # self.deviceStatus.setText(1,str(self.device.deviceStatus))

        self.motionDownsample.setText(1,str(self.device.motionDownsample))
        # self.torque.setText(1,str(self.device.torqueType))
        # self.controller.setText(1,str(self.device.controlType))
        self.update()
        self.blockSignals(False)

    def setTorqueMode(self, value):
        self.blockSignals(True)
        if value == SimpleFOCDevice.VOLTAGE_TORQUE:
            self.selectorTorque.setCurrentIndex(0)
        elif value == SimpleFOCDevice.DC_CURRENT_TORQUE:
            self.selectorTorque.setCurrentIndex(1)
        elif value == SimpleFOCDevice.FOC_CURRENT_TORQUE:
            self.selectorTorque.setCurrentIndex(2)
        self.blockSignals(False)

    def changeTorque(self):
        index = self.selectorTorque.currentIndex()
        if index == 0:
            self.device.sendTorqueType(SimpleFOCDevice.VOLTAGE_TORQUE)
        elif index == 1:
            self.device.sendTorqueType(SimpleFOCDevice.DC_CURRENT_TORQUE)
        elif index == 2:
            self.device.sendTorqueType(SimpleFOCDevice.FOC_CURRENT_TORQUE)

    def setEnabledDisabled(self, value):
        self.blockSignals(True)
        if value == 0:
            self.selectStatus.setCurrentIndex(0)
        elif value == 1:
            self.selectStatus.setCurrentIndex(1)
        self.blockSignals(False)

    def changeStatus(self):
        index = self.selectStatus.currentIndex()
        if index == 0:
            self.device.sendDeviceStatus(0)
        elif index == 1:
            self.device.sendDeviceStatus(1)
        
    def setModCenter(self,value):
        self.blockSignals(True)
        self.selectModCenter.setCurrentIndex(value)
        self.blockSignals(False)
        
    def changeModCenter(self):
        index = self.selectModCenter.currentIndex()
        if index == 0:
            self.device.sendModulationCentered(0)
        elif index == 1:
            self.device.sendModulationCentered(1)

    def setModType(self, value):
        self.blockSignals(True)
        if value == SimpleFOCDevice.SINE_PWM:
            self.selectModulation.setCurrentIndex(0)
        elif value == SimpleFOCDevice.SPACE_VECTOR_PWM:
            self.selectModulation.setCurrentIndex(1)
        elif value == SimpleFOCDevice.TRAPEZOIDAL_120:
            self.selectModulation.setCurrentIndex(2)
        elif value == SimpleFOCDevice.TRAPEZOIDAL_150:
            self.selectModulation.setCurrentIndex(3)
        self.blockSignals(False)

    def changeModType(self):
        index = self.selectModulation.currentIndex()
        if index == 0:
            self.device.sendModulationType(SimpleFOCDevice.SINE_PWM)
        elif index == 1:
            self.device.sendModulationType(SimpleFOCDevice.SPACE_VECTOR_PWM)
        elif index == 2:
            self.device.sendModulationType(SimpleFOCDevice.TRAPEZOIDAL_120)
        elif index == 3:
            self.device.sendModulationType(SimpleFOCDevice.TRAPEZOIDAL_150)

    def setControlLopMode(self, value):
        self.blockSignals(True)
        if value == SimpleFOCDevice.TORQUE_CONTROL:
            self.selectorControlLoop.setCurrentIndex(0)
        elif value == SimpleFOCDevice.VELOCITY_CONTROL:
            self.selectorControlLoop.setCurrentIndex(1)
        elif value == SimpleFOCDevice.ANGLE_CONTROL:
            self.selectorControlLoop.setCurrentIndex(2)
        elif value == SimpleFOCDevice.VELOCITY_OPENLOOP_CONTROL:
            self.selectorControlLoop.setCurrentIndex(3)
        elif value == SimpleFOCDevice.ANGLE_OPENLOOP_CONTROL:
            self.selectorControlLoop.setCurrentIndex(4)
        self.blockSignals(False)

    def changeControlLoop(self):
        index = self.selectorControlLoop.currentIndex()
        if index == 0:
            self.device.sendControlType(SimpleFOCDevice.TORQUE_CONTROL)
        elif index == 1:
            self.device.sendControlType(SimpleFOCDevice.VELOCITY_CONTROL)
        elif index == 2:
            self.device.sendControlType(SimpleFOCDevice.ANGLE_CONTROL)
        elif index == 3:
            self.device.sendControlType(SimpleFOCDevice.VELOCITY_OPENLOOP_CONTROL)
        elif index == 4:
            self.device.sendControlType(SimpleFOCDevice.ANGLE_OPENLOOP_CONTROL)
    
    def connectionStateChanged(self, connectionFlag):
        if connectionFlag is True:
            self.setEnabled(True)
        else:
            self.setEnabled(False)
