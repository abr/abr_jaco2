# -*- coding: utf-8 -*-
"""

This is a temporary script file.
"""

import demo11_noncompliant
import demo12_compliant
import demo13_adaptive
import demo21_compliant_reach
import demo22_adaptive_reach
import demo23_adaptive_reach_learned
import demo31_compliant_tool
import demo32_adaptive_hammer
import demo33_adaptive_hammer_learned

print("Welcome to the main ABR demo script.\n Choose a demo to run:\n")

print("0. Intro script (three targets)")
print("1. Demo 1.1 - Non-compliant")
print("2. Demo 1.2 - Compliant")
print("3. Demo 1.3 - Adaptive")
print("4. Demo 2.1 - Compliant reach")
print("5. Demo 2.2 - Adaptive reach first")
print("6. Demo 2.3 - Adaptive reach learned")
print("7. Demo 3.1 - Compliant tool")
print("8. Demo 3.2 - Adaptive tool")
print("9. Demo 3.3 - Adaptive tool learned")

while True:
    try:
        demo_number=int(input('Demo number:'))

    except ValueError:
        print("Not a valid demo number.")
    
    if demo_number == 0:
        demo0_introduction.main()
    elif demo_number == 1:
        demo11_noncompliant.main()
    elif demo_number == 2:
        demo12_compliant.main()
    elif demo_number == 3:
        demo13_adaptive.main()        
    elif demo_number == 4:
        demo21_compliant_reach.main()        
    elif demo_number == 5:
        demo22_adaptive_reach.main()        
    elif demo_number == 6:
        demo23_adaptive_reach_learned.main()        
    elif demo_number == 7:
        demo31_compliant_tool.main()        
    elif demo_number == 8:
        demo32_adaptive_hammer.main()        
    elif demo_number == 9:
        demo33_adaptive_hammer_learned.main()        
      
                