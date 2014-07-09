#include <StackArray.h>

StackArray <int> stack;

/* ====================================
 ================ SETUP ===============
 ====================================== */
void setup() {
  Serial.begin(9600);
  
//  stack = new StackArray();
  
  // add values 0 to 9 to stack
  for (int i = 0; i < 10; i++) {
    stack.push(i);
  }
  
  Serial.print("peek: "); Serial.println(stack.peek());
  stack.pop();
  Serial.print("peek: "); Serial.println(stack.peek());
}

/* ====================================
 ================ LOOP ================
 ====================================== */
void loop() {
  
}

