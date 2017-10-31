# Branch Note #
Objective is to extend the InformationObserver class creating multiple monitor class that each monitor a specific parameter of the rover.
<br>

Initial class implementation completed:
  * PressureMonitor
  * LocationMonitor
  * DistanceMonitor 
  * DirectionMonitor
  * BatteryMonitor
  * TemperatureMonitor
  * TimeMonitor
  * CameraStatusMonitor
  <br>
  
Added a `getJSON(File, key)` method in InformationObserver that parse the json File and then returns an Object `value` of the specified key
in the json file. 
