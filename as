USE PlantDB;
GO

SELECT TOP 100 *
FROM dbo.PurgeLogDetail
ORDER BY RunAt DESC, TelemetryTS;
