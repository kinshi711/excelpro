-- 例：対象は dbo.LineData
-- 目的：実行時点の「今日」(JST) までの全データを削除
--       ＝「明日 0:00 JST」より前の行を全削除
CREATE OR ALTER PROCEDURE dbo.usp_Purge_LineData_UptoToday
    @BatchSize int = 100000,  -- 小分け削除の1チャンク件数
    @SleepMs   int = 200,     -- チャンク間の休止(0～999ms)
    @UseUtc    bit = 1        -- [TIMESTAMP] が UTC保存=1 / JST等ローカル保存=0
AS
BEGIN
    SET NOCOUNT ON;
    SET DEADLOCK_PRIORITY LOW;

    -- 安全のため Sleep を 0～999 に丸める
    IF @SleepMs < 0 SET @SleepMs = 0;
    IF @SleepMs > 999 SET @SleepMs = 999;

    -- JST“今日”の境界を作る
    -- @UseUtc=1（UTC保存）の場合：UTC→JST(+9h)で今日/明日0時を出してからUTCへ戻す
    -- @UseUtc=0（ローカル保存）の場合：サーバーローカルの今日/明日0時をそのまま使う
    DECLARE @nowJst      datetime2(0) =
        CASE WHEN @UseUtc=1
             THEN DATEADD(hour, 9, SYSUTCDATETIME())
             ELSE SYSDATETIME()
        END;
    DECLARE @tomorrow0Jst datetime2(0) = DATEADD(day, 1, CAST(CONVERT(date, @nowJst) AS datetime2(0)));

    DECLARE @cutoff datetime2(0) =
        CASE WHEN @UseUtc=1
             THEN DATEADD(hour, -9, @tomorrow0Jst)  -- 明日0:00(JST) を UTC に戻す
             ELSE @tomorrow0Jst                     -- 明日0:00(ローカル)
        END;

    DECLARE @delay time(3) = TIMEFROMPARTS(0,0,0,@SleepMs,3);
    DECLARE @totalDeleted bigint = 0;

    WHILE 1=1
    BEGIN
        ;WITH cte AS (
            SELECT TOP (@BatchSize) *
            FROM dbo.LineData WITH (READPAST, ROWLOCK)
            WHERE [TIMESTAMP] < @cutoff
            ORDER BY [TIMESTAMP] ASC
        )
        DELETE FROM cte;

        DECLARE @rc int = @@ROWCOUNT;
        SET @totalDeleted += @rc;

        IF @rc < @BatchSize BREAK;       -- 残りが少ない→終了
        IF @SleepMs > 0 WAITFOR DELAY @delay;  -- 負荷平準化
    END

    -- 実行結果（ログ確認用）
    SELECT @totalDeleted AS DeletedRows, @cutoff AS CutoffBoundary;
END
GO
