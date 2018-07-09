UTC     = datetime(2004,5,14,16,43,0); % UTC Time
JD_UTC  = jd(UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second); % [days]
d_AT    = 32.0;         % [sec] From Astronomical Almanac 2004:K9
d_UT1   = -0.463326;   % [sec] April 6 2004 (Source: IERS EOPCO4)

[ut1, tut1, jdut1, utc, tai, tt, ttt, jdtt, tdb, ttdb, jdtdb, tcg, jdtcg, tcb, jdtcb] ...
         = convtime (UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second,d_UT1, d_AT);
 
fprintf(1,'ut1 %8.6f tut1 %16.12f jdut1 %18.11f\n',ut1,tut1,jdut1 );
        fprintf(1,'utc %8.6f\n',utc );
        fprintf(1,'tai %8.6f\n',tai );
        fprintf(1,'tt  %8.6f ttt  %16.12f jdtt  %18.11f\n',tt,ttt,jdtt );
        fprintf(1,'tdb %8.6f ttdb %16.12f jdtdb %18.11f\n',tdb,ttdb,jdtdb );



     