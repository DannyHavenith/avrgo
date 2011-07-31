#!/usr/bin/perl
while (<>)
{
    if (/(.)(.)(.)(.)\s+(.)(.)(.)(.)\s+(.)(.)(.)(.)\s+(.)(.)(.)(.)\s+(.*)/)
    {
    ++$count;
    $ins =  $17;
    chomp $ins;
    print "    " . $ins . ",\n";
    }
}
print $count;
