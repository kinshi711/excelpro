CREATE TABLE dbo.PurgeLogDetail (
    RunAt        datetime2(3) NOT NULL,  -- バッチ実行時刻
    Cutoff       datetime2(3) NOT NULL,  -- この境界より古いデータを削除した
    TelemetryId  bigint       NOT NULL,  -- 削除したTelemetry側の主キー
    TelemetryTS  datetime2(3) NOT NULL   -- 削除した行の時刻
    -- 必要なら他の列も後からADDできます (例: 設備ID, チャンネル番号 等)
);
