int var1=20;            // in .data
static int var2=20;     // in .data  
const int var3=20;      // in .rodata
int var4;               //in .bss

int main(void)
{
  static int var5 =55; //in .data
  static int var6 =0; //in  .bss
}