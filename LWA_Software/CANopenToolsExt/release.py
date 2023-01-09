# -*- coding: latin-1 -*-
#######################################################################
#
## \file
#  \section canopentools_release_py_general General file information
#
#    /author   Dirk Osswald
#    \date     2012-04-04
#
#  \brief  
#    Definition and documentation of the project name and the release
#    name ("version") of the package. The doxygen comments of the
#    release name serve as the change log of the project. 
#
#  \section canopentools_release_py_copyright Copyright
#
#  Copyright (c) 2012 SCHUNK GmbH & Co. KG
#
#  <HR>
#  \internal
#
#    \subsection canopentools_release_py_details SVN related, detailed file specific information:
#      $LastChangedBy: Osswald2 $
#      $LastChangedDate: 2014-09-16 17:34:11 +0200 (Tue, 16 Sep 2014) $
#      \par SVN file revision:
#        $Id: release.py 12227 2014-09-16 15:34:11Z Osswald2 $
#
#  \subsection canopentools_release_py_changelog Changelog of this file:
#      \include release.py.log
#
#######################################################################

#######################################################################
## \anchor canopentools_release_py_python_vars
#  \name   Python specific variables
#  
#  Some definitions that describe the module for python.
#
#  @{

__doc__       = """Definition and documentation of the project name and the release name ("version") for CANopenTools package""" #@ReservedAssignment
__author__    = "Dirk Osswald: dirk.osswald@de.schunk.com"
__url__       = "http://www.schunk.com"
__version__   = "$Id: release.py 12227 2014-09-16 15:34:11Z Osswald2 $"
__copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG"

#  end of doxygen name group canopentools_release_py_python_vars
#  @}
######################################################################

######################################################################
# Define some variables

## \brief Name of the software project. 
#
#  \anchor project_name_canopentools
#  The name of the "CANopenTools" package.
#
#    \internal
#    \remark 
#    - This name is extracted by the setup.py and used:
#      - As name of the project within the generated zip archive.
#    - The name should \b NOT contain spaces!
#
PROJECT_NAME = "CANopenTools" 

## \brief Release name of the whole software project (a.k.a. as the \e "version" of the project). 
#
#    \anchor project_release_canopentools
#    The release name of the "CANopenTools" project. The doxygen comment below
#    contains the changelog of the project. 
#
#    A suffix of "-dev" indicates a work in progress, i.e. a not yet finished release.
#    A suffix of "-a", "-b", ... indicates a bugfix release.
#
#    From newest to oldest the releases have the following names and features:
#
#    - 1.0.2.8 2014-09-16
#      - Minor improvements with unreferenced modules for COT_ProfilePositionMode, see bug 1604
#      - Refactured sub packages smpcom and canopen, now both in separate pyschunk package
#      - For "-n 0" scanning for nodes is now done for all COT_* tools
#
#    - 1.0.2.7 2014-09-08
#      - Added support for viewing new build information build_compiler, build_host, build_user
#        in COT_GetErrorDetails COT_GetManufacturerDetails and LWA_Tool (requires Firmware 0.65)
#      - Moved COT_SetUser tool from internal only to external since it is usefull for enduser as well.
#      - Made CANopenTools work with new easily distributable pytools and pyschunk packages
#      - The usage of the first SDO server can now be enforced via command line parameter / environment variable with all tools
#      - Updated ntcan interface module to PyNTCAN-2.1.0.win32-py2.7.exe 
#      - Updated python to v2.7.8
#
#    - 1.0.2.6 2014-06-13
#      - Added COT_RecordCANErrors for internal
#      - minor improvements to LWA_Tool: enable of all axes is now with delay as requested
#      - Added support for secondary SDO-Server. This allows to use the COT in parallel
#        to other SDO clients (e.g. KEBA-Control)
#      - refactored ThreadWithExc class to pytools for easys reuse in SDHx miniterm
#      - Added sdo.py script for easy command line access to object dictionary via SDOs
# 
#    - 1.0.2.5 2014-02-13
#      - Added online/offline detection of modules
#      - Added snapshot feature to simplify remote diagnosis, available in context menue
#
#    - 1.0.2.4 2014-01-14
#      - corrected setting of mode of operation to better work with new behaviour of operation_enabled 
#        bit of firmware
#      - corrected position and status output in COT_ReferenceTool.py for SMP modules
#      - made SMPT_ReferenceToolERB.py work again
#      - refactored to generate zip archives in zip subdirectory
#      - refactored to load pictures for LWA_Tool from pic subdirectory
#      - refactored to be able to load pictures from new pyschunk/guistuff/pic
#      - Added Makfile to simplify creation of distros
#      - Added possibility to generate distros with PyInstaller, but that creates way larger archives so sticking to py2exe
#
#    - 1.0.2.3 2013-10-29
#      - completely rewrote esdcanopen.SDO.read to handle errors observed on MGS 
#        laptops, see bug 1426: Bug: COT_TestIndexPulse.exe bricht mit Fehlermeldung ab
#        https://192.168.101.101/mechatronik/show_bug.cgi?id=1426
#        SDO upload should now be way more robust
#      - Further improved LWA_Tool according to feedback from Bruno Fellhauer and Frank Friedrichs:
#        - first/last step of tutorial disables the invalid previouse/next step button
#        - displaying tutorial step in format n/N
#        - button unlock brake changes text whenn pressed to lock brake
#        - re-ordered "Record / Replay" label to match order of corresponding button
#        - Reordered gui: interactive move stuff is now on top, then state then rest
#        - Tooltip for statusbar shows some text in statusbar when shown
#        - fixed bug 'tutorial is shown on CTRL-F1' (enable Axis 1)
#        - made mousewheel scroll the scrollable data area
#        - Added explanation of red/orange color in status display
#
#    - 1.0.2.2 2013-10-25
#      - improved COT_TestIndexPulse: 
#        - added waitforcommutation search complete to avoid jerk on first movement
#        - missed index pulses on PC are no longer reported as major errors
#        - beep alarm is only rang once in the end in case of errors  
#      - simplified error handling when reading SDO responses
#
#    - 1.0.2.1 2013-10-17
#      - Rearranged interctive control buttons for better clearity
#      - added fast stop button
#      - added tutorial
#
#    - 1.0.1.12 2013-09-10
#      - Added COT_TestIndexPulse see bug 1407: Task: Testprogramm für Encoder https://192.168.101.101/mechatronik/show_bug.cgi?id=1407
#
#    - 1.0.1.11 2013-08-28
#      - Done bug 1362: Task: CANopenTools: Es fehlt ein Tool um Baudrate aller Arm-Module umzustellen https://192.168.101.101/mechatronik/show_bug.cgi?id=1362
#      - Done bug 1398: 1398: Task: LWA_Tool erweitern https://192.168.101.101/mechatronik/show_bug.cgi?id=1398
#        - Info to display can be selected (hide/unhide of categories)
#        - poses can be recorded/replayed/saved/loaded
#        - Added auto scroll bars if place is insufficient
#        - Made program more robust and more responsive
#      - Refactured gui stuff to simplify reuse
#
#    - 1.0.1.10 2013-08-07
#      - added new tools COT_CurrentMode, COT_SetKS_FeedForward, COT_SetUser
#      - improved error handling of SDO class, added convenience functions to read/write floats 
#      - restructured COT_InterpolatedPositionMode to simplify reuse
# 
#    - 1.0.1.9 2013-07-04
#      - improved icons
#      - bugfix bug 1361: Bug: CANopenTools SMPT_ResetToCANopen funktioniert nur unzuverlässig https://192.168.101.101/mechatronik/show_bug.cgi?id=1361
#      - improved tooltip of info message to include all previous messages with time
#
#    - 1.0.1.8 2013-06-25
#      - improved LWA_Tool for KUKA-demo
#
#    - 1.0.1.7 2013-06-11
#      - added LWA_Tool for GL-demo
#
#    - 1.0.1.6 2013-06-03
#      - translated COT_ReferenceTool to englisch
#      - added auto-generation of external variant of CANopenTools to setup.py 
#
#    - 1.0.1.5 2013-03-27
#      - made COT_ControllerTuning run as exe 
#      - made GUIs dissappear while waiting in util.ConfirmExit()
#
#    - 1.0.1.4 2012-12-04
#      - Improved COT_ControllerTuning (added delta_pos, improved coloring for changed values)
#      - Improved COT_GetTemperature (added SDO mode)
#      - Improved COT_Velocityode (added setting of deceleration for smooth change of direction, bugfix for bug 1070)
#      - Improved COT_ReferenceTool (added index pulse measurement, bugfix for bug 1220)
#      - Added COT_GetEEPROM (bugfix for bug 1221)
#
#    - 1.0.1.3 2012-10-24
#      - Added COT_Dereference
#      - Improved COT_ReferenceTool
#
#    - 1.0.1.2
#      - Added ramp profile to COT_InterpolatedPositionMode
#
#    - 1.0.1.1 
#      - Added COT_ProfilePositionMode
#      - Resolved Bug 1196: Bug: Root Passwort kann nicht eingegeben werden https://192.168.101.101/mechatronik/show_bug.cgi?id=1196
#      - Added COT_InterpolatedPositionMode
#      - Added COT_ControllerTuning
#      - Added COT_VelocityMode
#      - Added COT_ReferenceTool
#      - Improved COT_GetErrorDetails (report includes current date, firmware version, statusword)
#      - Improved COT_UnlockBrake (nodes must be operational)
#      - Added nmt.py
#      - Improved COT_GetManufacturerDetails to be able to scan for nodes
#
#    - 1.0.1.0 
#      - Added COT_GetCurrent
#      - Improved COT_GetErrorDetails, added code to convert error codes into human readable form
#      - Resolved Bug 1183: Bug: COT_GetTemperature.py liefert falsche Temperatur Werte https://192.168.101.101/mechatronik/show_bug.cgi?id=1183
#
#    - 1.0.0.9 2012-07-18
#      - Changed prefix of name of tools that work in SMP only from COP to SMPT
#      - Added SMPT_MeasureLimit
#      - Done Bug 1170: 1170: Bug: manche CANopen Tools funktionieren nicht für Release Firmware https://192.168.101.101/mechatronik/show_bug.cgi?id=1170
#
#    - 1.0.0.8
#      - Done Bug 1154: Task: Temperatur Logger für SMP https://192.168.101.101/mechatronik/show_bug.cgi?id=1154 
#
#    - 1.0.0.7
#      - Done Bug 1152: Task: Handling of logfile Parameter vereinheitlichen https://192.168.101.101/mechatronik/show_bug.cgi?id=1152
#      - Done Bug 1151: Task: Erstmalige Referenzierung für ERB vereinfachen https://192.168.101.101/mechatronik/show_bug.cgi?id=1151
#
#    - 1.0.0.6 2012-05-02
#      - Added forgotten COT_SetDebug to setup.py
#      - Added cmd-here.lnk for easy starting of Windows command prompts on users system
#      - Added setpath.bat for easy PATH modification from users command prompt
#      - Added --nocolor option to COT_GetDebugMessages
#      - bugfix: if plot.py cannot be found then NodeListener terminates itself
#      - Invalid temperatures from nodes are now printed as -1.0
#
#    - 1.0.0.5 2012-05-02
#      - Added GPD output to COT_GetTemperature
#      - Improved COT_ReferenceTool to send ACK before referencing
#
#    - 1.0.0.4 2012-04-17
#      - Added COT_UnlockBrake 
#      - Added COT_Logging 
#
#    - 1.0.0.3 2012-04-13
#      - renamed project to CANopenTools
#      - COT_GetDebugMessages (getplotcanopen) added from MotionToolCANopen
#      - heavily restructured code for simpler and clearer usage
#      - renamed user scripts with COT_ prefix
#      - COT_SetDebug and COT_GetState added
# 
#    - 1.0.0.2 2012-04-12
#      - ResetToCANopen added
#      - get_manufacturer_details added
#      - get_error_details added
#
#    - 1.0.0.1 2012-04-04
#      - ReferenceTool added.
#
#    - ? 
#      - initial inofficial releases for KEBA, T. Dischinger, R. Koch

PROJECT_RELEASE = "1.0.2.8" # py2exe can handle integers here only!
                            # E.G. "1.0.2.2alpha" yields: 
                            #   warning: py2exe: Version Info will not be included: could not parse version number '

## \brief Date of the release of the software project.
#
#    \anchor project_date_canopentools
#    The date of the release of the project. 
#
PROJECT_DATE = "2014-09-16"
