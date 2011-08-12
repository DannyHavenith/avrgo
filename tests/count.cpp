volatile unsigned short target;

int main()
{
    for( unsigned short count = 0; count < 10; ++count)
    {
     	target = count;
    }
    return 0;
}
