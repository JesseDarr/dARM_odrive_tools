$files = Get-ChildItem -Path "./" -File -Recurse 

$output = @()
foreach ($file in $files) { 
    $output += "`n`n`nFile: $($file.FullName)`n"
    $output += (Get-Content $file.FullName) -replace '///', 'xxx' 
}

$output | clip.exe