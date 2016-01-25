0.0:
 EdisonEclipseインストール
 http://downloadmirror.intel.com/25028/eng/iotdk_win_installer.exe
 WindowsPCとEdisonを接続
 Windowsドライバーのインストールが始まるが、ほとんど失敗するがそのまま放置
 ※インストール中に個々のWindowsドライバーの
   インストールを促すウィンドウが表示されるので「インストールします」を選択
   ※ドライバインストール失敗は無視でいいです
0.1:
 以下のサイトからyamlライブラリ（ yaml-0.1.5.tar.gz ）を取得してEdison内にコピーして
 http://pyyaml.org/wiki/LibYAML
0.2:
 Edison内にInstall
 root@edison:~# ./configure 
 root@edison:~# make 
 root@edison:~# make install
0.3:
 Edison内にInstallされた以下のライブラリをWindowsPC上にコピー
 src/.libs/libyaml.so	->WindowsPC-> C:\Intel\iotdk-ide-win\devkit-x86\sysroots\i586-poky-linux\lib 
 src/.libs/libyaml-0.so.2	->WindowsPC-> C:\Intel\iotdk-ide-win\devkit-x86\sysroots\i586-poky-linux\lib 
 src/.libs/libyaml-0.so.2.0.3	->WindowsPC-> C:\Intel\iotdk-ide-win\devkit-x86\sysroots\i586-poky-linux\lib 
0.4:
 予めEdison←microUSB→WindowsPCを接続して
 Windowsメニューから「Intel IoT Developer Kit」→「Eclipse」を選択し、
 起動されたEclipseのメニューから「Intel(R) IoT C/C++ project」選択
0.5:
 WindowsPC_Eclipseで以下の設定を行う
 ・ライブラリ指定
 1.「C/C++ Build」 
 2.「Settings」を選択 
 3.「Tools Settings」タブ 
 4.「C++ Linker」項目からLibrariesを選択 
 5.「Libraries(-l)」から、新規追加ボタン(+ボタン)で追加 
 ->pthread
 ->yaml
 ・マクロ指定
 1.「C/C++ Build」 
 2.「Settings」を選択 
 3.「Tools Settings」タブ 
 4.「 CrossG++Compiler 」項目からPreprocessorを選択 
 5.「Defined symbols(-D)」から、新規追加ボタン(+ボタン)で追加 
 ->NDEBUG
 ->_USRDLL
 -> GR001_EXPORTS
 -> YAML_DECLARE_STATIC
※ここまでは初回のみ

1:
 本Gitから得たファイル群を上記環境のsrcフォルダに展開してEclipseビルドを行う
