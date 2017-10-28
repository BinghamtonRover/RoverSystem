<h1>Branch note</h1>
Objective is to extend the InformationObserver class creating multiple monitor class that each monitor a specific parameter of the rover.
<br>

Initial class implementation completed:
  <li>PressureMonitor</li>
  <li>LocationMonitor</li>
  <li>DistanceMonitor</li>
  <br>
  
Added a getJSON(File, key) method in InformationObserver that parse the json File and then returns an Object value of the specified key. 
