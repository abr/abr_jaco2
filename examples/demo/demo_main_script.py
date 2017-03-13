# -*- coding: utf-8 -*-
"""

This is a temporary script file.
"""

import demo11_noncompliant
import demo12_compliant

print("Welcome to the main ABR demo script.\n Choose a demo to run:\n")

print("1. Demo 1.1 - Non-compliant")
print("2. Demo 1.2 - Compliant")

while True:
    try:
        demo_number=int(input('Demo number:'))

    except ValueError:
        print("Not a valid demo number.")
    
    if demo_number == 1:
        temp1.main()
    elif demo_number == 2:
        temp2.main()
        
        