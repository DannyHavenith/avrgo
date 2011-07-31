#!/usr/bin/perl
while (<>)
{
    if (/(.)(.)(.)(.)\s+(.)(.)(.)(.)\s+(.)(.)(.)(.)\s+(.)(.)(.)(.)\s+(.*)/)
    {
    $match[0] = $1;
    $match[1] = $2;
    $match[2] = $3;
    $match[3] = $4;
    $match[4] = $5;
    $match[5] = $6;
    $match[6] = $7;
    $match[7] = $8;
    $match[8] = $9;
    $match[9] = $10;
    $match[10] = $11;
    $match[11] = $12;
    $match[12] = $13;
    $match[13] = $14;
    $match[14] = $15;
    $match[15] = $16;
    $ins =  $17;
    $indent = '';
    for ($i = 0; $i < 8 - length( $ins); ++$i)
    {
        $indent .= ' ';
    }
    print "struct " .  $ins . $indent . ": public instruction< ";
    for ($i = 0; $i < 16; ++$i)
    {
        if ($match[$i] eq '.')
	{
    		$match[$i] = $prev[$i];
	}
        print $match[$i] . ",";
    }
    print "0>;\n";
    @prev = @match;
    }
}
