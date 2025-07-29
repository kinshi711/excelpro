using System;
using System.IO;
using System.Data;
using Iconics.Data; // GridWorX にデータを渡すため

public void LoadFileList(string filterPattern)
{
    // DataTable 作成（GridWorX Viewer に表示するため）
    DataTable table = new DataTable();
    table.Columns.Add("ファイル名");
    table.Columns.Add("更新日時");
    table.Columns.Add("サイズ (KB)");

    // 対象ディレクトリ（固定パス、テスト用）
    string folderPath = @"C:\Data\Test";   // 自分のテストフォルダに変更してください
    DirectoryInfo dir = new DirectoryInfo(folderPath);

    foreach (var file in dir.GetFiles("*.*")) // ← CSVに限定せず全ファイル対象
    {
        // フィルタ（ファイル名の部分一致）
        if (string.IsNullOrEmpty(filterPattern) ||
            file.Name.IndexOf(filterPattern, StringComparison.OrdinalIgnoreCase) >= 0)
        {
            table.Rows.Add(file.Name, file.LastWriteTime, (file.Length / 1024));
        }
    }

    // GridWorX Viewer のデータソースへ反映
    GridWorX.SetData("FileListGrid", table);
}
