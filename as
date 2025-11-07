SELECT
    TelemetryId,
    TelemetryTS,
    TagName
FROM dbo.PurgeLogDetail
WHERE RunAt = (SELECT MAX(RunAt) FROM dbo.PurgeLogDetail)
ORDER BY TelemetryTS;
