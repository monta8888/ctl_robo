0.0:
 EdisonEclipse�C���X�g�[��
 http://downloadmirror.intel.com/25028/eng/iotdk_win_installer.exe
 WindowsPC��Edison��ڑ�
 Windows�h���C�o�[�̃C���X�g�[�����n�܂邪�A�قƂ�ǎ��s���邪���̂܂ܕ��u
 ���C���X�g�[�����ɌX��Windows�h���C�o�[��
   �C���X�g�[���𑣂��E�B���h�E���\�������̂Łu�C���X�g�[�����܂��v��I��
   ���h���C�o�C���X�g�[�����s�͖����ł����ł�
0.1:
 �ȉ��̃T�C�g����yaml���C�u�����i yaml-0.1.5.tar.gz �j���擾����Edison���ɃR�s�[����
 http://pyyaml.org/wiki/LibYAML
0.2:
 Edison����Install
 root@edison:~# ./configure 
 root@edison:~# make 
 root@edison:~# make install
0.3:
 Edison����Install���ꂽ�ȉ��̃��C�u������WindowsPC��ɃR�s�[
 src/.libs/libyaml.so	->WindowsPC-> C:\Intel\iotdk-ide-win\devkit-x86\sysroots\i586-poky-linux\lib 
 src/.libs/libyaml-0.so.2	->WindowsPC-> C:\Intel\iotdk-ide-win\devkit-x86\sysroots\i586-poky-linux\lib 
 src/.libs/libyaml-0.so.2.0.3	->WindowsPC-> C:\Intel\iotdk-ide-win\devkit-x86\sysroots\i586-poky-linux\lib 
0.4:
 �\��Edison��microUSB��WindowsPC��ڑ�����
 Windows���j���[����uIntel IoT Developer Kit�v���uEclipse�v��I�����A
 �N�����ꂽEclipse�̃��j���[����uIntel(R) IoT C/C++ project�v�I��
0.5:
 WindowsPC_Eclipse�ňȉ��̐ݒ���s��
 �E���C�u�����w��
 1.�uC/C++ Build�v 
 2.�uSettings�v��I�� 
 3.�uTools Settings�v�^�u 
 4.�uC++ Linker�v���ڂ���Libraries��I�� 
 5.�uLibraries(-l)�v����A�V�K�ǉ��{�^��(+�{�^��)�Œǉ� 
 ->pthread
 ->yaml
 �E�}�N���w��
 1.�uC/C++ Build�v 
 2.�uSettings�v��I�� 
 3.�uTools Settings�v�^�u 
 4.�u CrossG++Compiler �v���ڂ���Preprocessor��I�� 
 5.�uDefined symbols(-D)�v����A�V�K�ǉ��{�^��(+�{�^��)�Œǉ� 
 ->NDEBUG
 ->_USRDLL
 -> GR001_EXPORTS
 -> YAML_DECLARE_STATIC
�������܂ł͏���̂�

1:
 �{Git���瓾���t�@�C���Q����L����src�t�H���_�ɓW�J����Eclipse�r���h���s��
