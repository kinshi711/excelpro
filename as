SELECT d.TelemetryId, d.TagName, d.TelemetryTS, d.DeleteFlg
FROM PlantDB.dbo.PurgeLogDetail AS d
WHERE d.RunAt = (SELECT MAX(RunAt) FROM PlantDB.dbo.PurgeLogDetail)
ORDER BY d.TelemetryTS;
