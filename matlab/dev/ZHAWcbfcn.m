function varout = ZHAWcbfcn(mode,option)
%ZHAWcbfcn callback function for ZHAW SLDRT library.
% Version R2024a with new Chirp for BuR
% Otto Fluder / ZHAW / IMS / fldr@zhaw.ch
if nargin==0
    mode = 'bode';
end
if nargin==1 
    option = 0;
end
blk = gcb;
try
    set_param('ZHAW_SLDRT_lib','lock','on');
end
if strcmp(mode,'openMask')
    open_system(blk,'Mask');
end
if strcmp(bdroot,'ZHAW_SLDRT_lib')
    return
end
switch mode
    case 'StartTrigger'
        blk_chirp = find_system(bdroot,'MaskType','bode_chirp');
        if ~isempty(blk_chirp)
            blk_chirp = blk_chirp{1};
        else
            blk_chirp = [];
        end
        if strcmp(get_param(bdroot,'SimulationStatus'),'stopped')
            if strcmp(get_param(bdroot,'SystemTargetFile'),'bur_grt.tlc')
                set_param(bdroot,'SimulationCommand','connect');
            else
                set_param(bdroot,'SimulationCommand','start')
            end
            set_param(blk_chirp,'MaskDisplay','disp(''Stop'')');
            pause(0.5)
            %set_param(blk_chirp,'Trig','0')
            %pause(0.1)
            set_param([blk,'/Trigger'],'Value','1')
            %set_param(blk_chirp,'Trig','1')
            set_param(blk,'Trig','1')
        else
            set_param(blk,'MaskDisplay','disp(''Start'')');
            set_param([blk_chirp,'/Trigger'],'Value','0')
            %set_param(blk_chirp,'Trig','0')
            set_param([blk,'/Trigger'],'Value','0')
            %set_param(blk,'Trig','0')
            if strcmp(get_param(bdroot,'SystemTargetFile'),'bur_grt.tlc')
                set_param(bdroot,'SimulationCommand','disconnect');
            else
                set_param(bdroot,'SimulationCommand','stop')
            end
        end
    case 'TrigChirpClb'
        set_param([blk,'/Trigger'],'Value','0')
        pause(0.1)
        set_param([blk,'/Trigger'],'Value','1')
        
%         if strcmp(get_param([blk,'/Trigger'],'Value'),'1')
%             set_param([blk,'/Trigger'],'Value','0')
%         else
%             set_param([blk,'/Trigger'],'Value','1')
%         end
    case 'SetPref'
        blk = 'ZHAW_SLDRT_lib/ Set Preferences';
        load_system('ZHAW_SLDRT_lib');
        set_param('ZHAW_SLDRT_lib','lock','off')
        set_param(blk,'device',getpref('ZHAW_SLDRT_lib','device'));
        set_param(blk,'connect',getpref('ZHAW_SLDRT_lib','connect'));
        set_param(blk,'couple',getpref('ZHAW_SLDRT_lib','couple'));
        set_param(blk,'funits',getpref('ZHAW_SLDRT_lib','funits'));
        set_param(blk,'aunits',getpref('ZHAW_SLDRT_lib','aunits'));
        set_param(blk,'punits',getpref('ZHAW_SLDRT_lib','punits'));
        set_param(blk,'unwrap',getpref('ZHAW_SLDRT_lib','unwrap'));
        %set_param(blk,'shiftphase',getpref('ZHAW_SLDRT_lib','shiftphase'));
        set_param(blk,'max_sampling_frequency',getpref('ZHAW_SLDRT_lib','max_sampling_frequency'));
        open_system(blk,'Mask')
    case 'namechangeGPA'
        blk_name = get_param(blk,'Name');
        blk_name = strrep(blk_name,' ','_');
        set_param([blk,'/To Workspace'],'VariableName',[blk_name,'_td']);
    case 'copyScope'
        set_param(blk,'LinkStatus','none');
        set_param(blk,'LimitDataPoints','off');
        set_param(blk,'SampleTime','Ts');
        set_param(blk,'DataLogging','on');
        set_param(blk,'DataLoggingSaveFormat','Structure With Time');
        blk_name = 'data';
        k = 0;
        iserror = true;
        while iserror
            try
                set_param(blk,'Name',blk_name);
                iserror = false;
            catch
                k = k+1;
                blk_name = ['data',num2str(k)];
                iserror = true;
            end
        end    
    case 'namechangeScope'
        SaveName = get_param(blk,'Name');
        SaveName = strrep(SaveName,' ','_');
        set_param(blk,'DataLoggingVariableName',SaveName);
    case 'copyToWS'
        set_param(blk,'LinkStatus','none');
        set_param(blk,'SaveFormat','StructureWithTime');
        set_param(blk,'SampleTime','Ts');
        set_param(bdroot,'ReturnWorkspaceOutputs','off');
        blk_name = 'data';
        k = 0;
        iserror = true;
        while iserror
            try
                set_param(blk,'Name',blk_name);
                iserror = false;
            catch
                k = k+1;
                blk_name = ['data',num2str(k)];
                iserror = true;
            end
        end
    case 'namechangeToWS'
        SaveName = get_param(blk,'Name');
        SaveName = strrep(SaveName,' ','_');
        set_param(blk,'VariableName',SaveName);
    case 'SetupSLDRT'
        set_param(bdroot,'ExtModeMexArgs','');
        if strcmp(get_param(bdroot,'SimulationStatus'),'stopped')
            Install_NI_Device;
            set(0,'ShowHiddenHandles','on');
            hf = get(0,'CurrentFigure');
            if ~isempty(hf) & strcmp(get(hf,'Tag'),'SIMULINK_SIMSCOPE_FIGURE')
                set(hf,'KeyPressFcn','if double(get(gcf,''CurrentCharacter''))==20, if strcmp(get(gcf,''Toolbar''),''figure''),set(gcf,''ToolBar'',''none''),else,set(gcf,''ToolBar'',''figure''),end,end')
            end
            set(0,'ShowHiddenHandles','off');
            
            Ts = eval(get_param(gcb,'Ts'));
            fs_max = eval(getpref('ZHAW_SLDRT_lib','max_sampling_frequency','20000'));
            if strcmp(get_param(gcb,'ext'),'with IO-Hardware / real-time') & (Ts<1/fs_max)
                Ts = 1/fs_max;
                set_param(gcb,'Ts',num2str(1/fs_max))
                warndlg(['Sample time too smal (Ts>=',num2str(1/fs_max),'s).'])
            end
            assignin('base','Ts',Ts);
            fname = get_param(bdroot,'FileName');
            if exist(fname,'file')
                cd(fileparts(fname));
            end
            set_param(bdroot,'SolverType','Fixed-step');
            if strcmp(get_param(gcb,'ext'),'with IO-Hardware / real-time')
                set_param(bdroot,'SimulationMode','external')
            else
                set_param(bdroot,'SimulationMode','normal')
            end
            if strcmp(get_param(gcb,'ext'),'without IO-Hardware / non real-time')
                set_param([bdroot,'/ Setup SLDRT/Synchronization'],'value','0');
            else
                set_param([bdroot,'/ Setup SLDRT/Synchronization'],'value','1');
            end
            set_param(bdroot,'Solver',get_param(gcb,'solvertyp'));
            sldrtconfigset(bdroot);
            set_param(bdroot,'UnconnectedInputMsg','none');
            set_param(bdroot,'UnconnectedOutputMsg','none');
            set_param(bdroot,'UnconnectedLineMsg','none');
            set_param(bdroot,'DiscreteInheritContinuousMsg','none');
            set_param(bdroot,'SolverPrmCheckMsg','none');
            set_param(bdroot,'SaveTime','off');
            set_param(bdroot,'SaveOutput','off');
            set_param(bdroot,'SaveState','off');
            set_param(bdroot,'ReturnWorkspaceOutputs','off');
            
            set_param(bdroot,'FixedStep','Ts');
            Te = evalin('base',get_param(bdroot,'StopTime'));
            if isinf(Te)
                N_samp = 1e7;
            else
                N_samp = max(ceil(Te/Ts+1),1e7);
            end
            set_param(bdroot,'ExtModeTrigDuration',num2str(N_samp));
            blk_scope = find_system(bdroot,'BlockType','Scope');
            for k = 1:length(blk_scope)
                set_param(blk_scope{k},'LimitDataPoints','off')
                %set_param(blk_scope{k},'SaveToWorkspace','off')
            end
            ind = strfind(gcb,'/');
            str = gcb;
            InitFcn = ['Ts=eval(get_param([bdroot,''',str(ind(1):end),'''],''Ts''));'];
            PostLoadFcn = ['try' char(10) ...
                'if isempty(which(''ZHAWcbfcn''));' char(10) ...
                'callback_fid07124=fopen(''ZHAWcbfcn.m'',''w'');' char(10) ...
                '[~]=fprintf(callback_fid07124,''%s'',get_param([bdroot,''/ Setup SLDRT''],''ClipboardFcn''));' char(10) ...
                '[~]=fclose(callback_fid07124);' char(10) ...
                'setpref(''ZHAW_SLDRT_lib'',''cb_fcn'',which(''ZHAWcbfcn''))' char(10) ...
                'clear callback_fid07124' char(10) ...
                'end' char(10) ...
                'catch' char(10) ...
                'clear callback_fid07124' char(10) ...
                'end' char(10) ...
                'Ts=eval(get_param([bdroot,''',str(ind(1):end),'''],''Ts''));'];
            set_param(bdroot,'CloseFcn','try ZHAWcbfcn(''CloseFcn'');end');
            set_param(bdroot,'InitFcn',InitFcn);
            set_param(bdroot,'PostLoadFcn',PostLoadFcn);
            Ts = str2num(get_param(blk,'Ts'));
            if strcmp(get_param(blk,'ext'),'with IO-Hardware / real-time')
                set_param(blk,'FontAngle','normal')
            else
                set_param(blk,'FontAngle','italic')
            end
        end
    case 'CloseFcn'
        blk = find_system(bdroot,'MaskType','SLDRT Setup');
%         if strcmp(get_param(blk,'close_fcn'),'on')
%             rpf = get_param(bdroot,'FileName');
%             rp = fileparts(rpf);
%             st = rmdir(fullfile(rp,'slprj'),'s');
%             st = rmdir(fullfile(rp,[bdroot,'_sldrt_win32']),'s');
%             st = rmdir(fullfile(rp,[bdroot,'_sldrt_win64']),'s');
%             f1 = fullfile(rp,[bdroot,'.rxw32']);
%             f2 = fullfile(rp,[bdroot,'.rxw64']);
%             if exist(f1,'file'), delete(f1); end;
%             if exist(f2,'file'), delete(f2); end;
%             rp = cd;
%             st = rmdir(fullfile(rp,'slprj'),'s');
%             st = rmdir(fullfile(rp,[bdroot,'_sldrt_win32']),'s');
%             st = rmdir( fullfile(rp,[bdroot,'_sldrt_win64']),'s');
%             f1 = fullfile(rp,[bdroot,'.rxw32']);
%             f2 = fullfile(rp,[bdroot,'.rxw64']);
%             f3 = getpref('ZHAW_SLDRT_lib','cb_fcn','');
%             setpref('ZHAW_SLDRT_lib','cb_fcn','');
%             if exist(f1,'file'), delete(f1); end;
%             if exist(f2,'file'), delete(f2); end;
%             if exist(f3,'file'), delete(f3); end;
%         end
    case 'CopyIOblock'
        set_param(blk,'LinkStatus',getpref('ZHAW_SLDRT_lib','LibraryLinkStatus','none'));
        %set_param(blk,'OpenFcn','open_system(gcb,''Mask'')')
        type = getdeviceinfo('type');
        dev = getpref('ZHAW_SLDRT_lib','device','PCIe-6321');
        connect = getpref('ZHAW_SLDRT_lib','connect','BNC Terminal');
        if strcmp(connect,'BNC Terminal')
            couple = 'differential';
        else
            couple = getpref('ZHAW_SLDRT_lib','couple','differential');
        end
        setpref('ZHAW_SLDRT_lib','couple',couple);
        set_param(gcb,'device',dev)
        set_param(gcb,'connect',connect)
        if strcmp(type,'AI')
            set_param(gcb,'couple',couple)
        end
        MaskType = getdeviceinfo('MaskType');
        N = length(find_system(bdroot,'LookUnderMasks','all','MaskType',MaskType));
        %if (N>1) && ~strcmp(MaskType,'NI Counter Input') && ~strcmp(MaskType,'NI PWM Output')
        if N>1
            warndlg(['Duplicate ',type,' block. Use only one block per type. For more channels, use parameter settings.'])
            name = ['ERROR DUPLICATE ',type,' BLOCKS!'];
            for k=2:N
                name = [name,' '];
            end
            set_param(gcb,'Name',name)
        else
            name = [type,' / ',dev];
        end
        %set_param(gcb,'Name',name)
        set_param(gcb,'MaskDisplay',['disp(''NI\n',dev,''')']);
    case 'SetChannelNumber'
        [type,name,type_out,dev,N_CH,ch_list,offset] = getdeviceinfo;
        vis = get_param(gcb,'MaskVisibilities');
        switch type
            case {'AO','PWM','DI','DO'}
                vis((1:N_CH)+2)={'on'};
                vis((N_CH+1+2):6)={'off'};
            case {'ENC','CTR'}
                vis((1:N_CH)+3)={'on'};
                vis((N_CH+1+3):7)={'off'};
        end
        set_param(gcb,'MaskVisibilities',vis);
    case 'SetConnectorBox'
        set_param(gcb, 'LinkStatus', 'inactive')
        [type,name,type_out,dev,N_CH,ch_list,offset] = getdeviceinfo;
        vis = get_param(gcb,'MaskVisibilities');
        if strcmp(type,'AI')
            if strcmp(get_param(blk,'connect'),'BNC Terminal')
                vis(3)={'off'};
                set_param(blk,'couple','differential');
            else
                vis(3)={'on'};
            end
        end
        set_param(gcb,'MaskVisibilities',vis);
        MaskPrompts = get_param(gcb,'MaskPrompts');
        switch type
            case 'AI'
                list = [68,33,65,30,28,60,25,57,34,66,31,63,61,26,58,23,67,32,64,29,27,59,24,56,67,32,64,29,27,59,24,56];
                for k=4:2:18
                    ch = k/2-1;
                    if getdeviceinfo('differential')
                        switch get_param(blk,'connect')
                            case 'Banana Terminal'
                                MaskPrompts{k}=[name,num2str(ch),'-',name,num2str(ch+8)];
                            case 'BNC Terminal'
                                MaskPrompts{k}=[name,num2str(ch-1)];
                            case 'Screw Terminal'
                                MaskPrompts{k}=[name,num2str(ch-1),' (SIG+Pin',num2str(list(ch)),' / SIG-Pin',num2str(list(ch+8)),')'];
                        end
                    else
                        switch get_param(blk,'connect')
                            case 'Banana Terminal'
                                MaskPrompts{k}=[name,num2str(ch),'-GND'];
                            case 'BNC Terminal'
                                MaskPrompts{k}=[name,num2str(ch-1)];
                            case 'Screw Terminal'
                                MaskPrompts{k}=[name,num2str(ch-1),' (SIG-Pin',num2str(list(ch)),' / GND-Pin',num2str(list(ch+16)),')'];
                        end
                    end
                end
                for k=5:2:19
                    ch = (k-3)/2+8;
                    switch get_param(blk,'connect')
                        case 'Banana Terminal'
                            MaskPrompts{k}=[name,num2str(ch),'-GND'];
                        case 'BNC Terminal'
                            MaskPrompts{k}=[name,num2str(ch-1)];
                        case 'Screw Terminal'
                            MaskPrompts{k}=[name,num2str(ch-1),' (SIG-Pin',num2str(ch),' / GND-Pin',num2str(ch+8),')'];
                    end
                end
            case 'AO'
                list = [22,21,22,21,55,54,55,54];
                for k=3:6
                    ch = k-2;
                    switch get_param(blk,'connect')
                        case 'Banana Terminal'
                            MaskPrompts{k}=[name,num2str(ch)];
                        case 'BNC Terminal'
                            MaskPrompts{k}=[name,num2str(ch-1)];
                        case 'Screw Terminal'
                            MaskPrompts{k}=[name,num2str(ch-1),' (SIG-Pin',num2str(list(ch)),' / GND-Pin',num2str(list(ch+4)),')'];
                    end
                    if N_CH==4
                        if ch<=2
                            MaskPrompts{k}=[MaskPrompts{k},' (',name,num2str(ch-1),' Connector0)'];
                        else
                            MaskPrompts{k}=[MaskPrompts{k},' (',name,num2str(ch-3),' Connector1)'];
                        end
                    end
                end
            case 'ENC'
                list = [37,42,11,6,45,46,43,38,3,41,10,5,36,7,35,4];
                for k=4:7
                    ch = k-3;
                    switch get_param(blk,'connect')
                        case 'Banana Terminal'
                            MaskPrompts{k}=[name,num2str(ch)];
                        case 'BNC Terminal'
                            MaskPrompts{k}=[name,num2str(ch-1)];
                        case 'Screw Terminal'
                            MaskPrompts{k}=['CTR',num2str(ch-1),' (A-Pin',num2str(list(ch)),' / B-Pin',num2str(list(ch+4)),' / Z-Pin',num2str(list(ch+8)),' / GND-Pin',num2str(list(ch+12)),')'];
                    end
                end
            case 'CTR'
                list = [37,42,11,6,45,46,43,38,3,41,10,5,36,7,35,4];
                for k=5:8
                    ch = k-4;
                    switch get_param(blk,'connect')
                        case 'Banana Terminal'
                            switch get_param(blk,'mode')
                                case {'Count rising edge','Count falling edge'}
                                    MaskPrompts{k}=[name,num2str(ch),' CHA'];
                                otherwise
                                    MaskPrompts{k}=[name,num2str(ch),' CHZ'];
                            end
                        case 'BNC Terminal'
                            MaskPrompts{k}=[name,num2str(ch-1)];
                        otherwise
                            switch get_param(blk,'mode')
                                case {'Count rising edge','Count falling edge'}
                                    MaskPrompts{k}=['CTR',num2str(ch-1),' (SOURCE-Pin',num2str(list(ch)),' / GND-Pin',num2str(list(ch+12)),')'];
                                otherwise
                                    MaskPrompts{k}=['CTR',num2str(ch-1),' (GATE-Pin',num2str(list(ch+8)),' / GND-Pin',num2str(list(ch+12)),')'];
                            end
                    end
                end
            case 'DI'
                list_HIGH = [52 17 49 47 19 51 16 48];
                list_GND = [53 18 13 12 9 50 15 7];
                for k=3:10
                    ch = k-2;
                    switch get_param(blk,'connect')
                        case 'Banana Terminal'
                            MaskPrompts{k}=['DI',num2str(ch)];
                        case 'BNC Terminal'
                            MaskPrompts{k}=['DI',num2str(ch-1)];
                        case 'Screw Terminal'
                            MaskPrompts{k}=['P0.',num2str(ch-1),' (High-Pin',num2str(list_HIGH(ch)),' / GND-Pin',num2str(list_GND(ch)),')'];
                    end
                end
            case 'DO'
                list_HIGH = [52 17 49 47 19 51 16 48];
                list_GND = [53 18 13 12 9 50 15 7];
                for k=3:10
                    ch = k-2;
                    switch get_param(blk,'connect')
                        case 'Banana Terminal'
                            MaskPrompts{k}=['DO',num2str(ch)];
                        case 'BNC Terminal'
                            MaskPrompts{k}=['DO',num2str(ch-1)];
                        case 'Screw Terminal'
                            MaskPrompts{k}=['P0.',num2str(ch-1),' (High-Pin',num2str(list_HIGH(ch)),' / GND-Pin',num2str(list_GND(ch)),')'];
                    end
                end
            case 'PWM'
                list = [2,40,1,39,36,7,35,4];
                for k=4:7
                    ch = k-3;
                    switch get_param(blk,'connect')
                        case 'Banana Terminal'
                            MaskPrompts{k}=[name,num2str(ch)];
                        case 'BNC Terminal'
                            MaskPrompts{k}=[name,num2str(ch-1)];
                        case 'Screw Terminal'
                            MaskPrompts{k}=['CTR',num2str(ch-1),' (SIG-Pin',num2str(list(ch)),' / GND-Pin',num2str(list(ch+4)),')'];
                    end
                end
        end
        set_param(gcb,'MaskPrompts',MaskPrompts);
    case 'SetChannelCoupling'
        set_param(gcb, 'LinkStatus', 'inactive')
        vis = get_param(gcb,'MaskVisibilities');
        if getdeviceinfo('differential')
            vis(5:2:19)={'off'};
        else
            vis(5:2:19)={'on'};
        end
        set_param(gcb,'MaskVisibilities',vis);
    case 'SetEncoderReset'
        reset = get_param(blk,'reset');
        if strcmp(reset,'on')
            set_param(gcb,'MaskDisplay','disp(''Encoder\nInput\nIndex Reset On'')');
            set_param([gcb,'/Encoder Input'],'IndexPulse','5');
        else
            set_param(gcb,'MaskDisplay','disp(''Encoder\nInput\nIndex Reset Off'')');
            set_param([gcb,'/Encoder Input'],'IndexPulse','1');
        end
    case 'get_ch_list'
        varout = getdeviceinfo('ch_list');
    case 'BlockUpdate'
        set_param(gcb, 'LinkStatus', 'inactive');
        [type,name,type_out,dev,N_CH,ch_list,offset] = getdeviceinfo;
        Install_NI_Device(dev);
        if type_out
            ports = find_system(blk,'LookUnderMasks','all','BlockType','Inport');
        else
            ports = find_system(blk,'LookUnderMasks','all','BlockType','Outport');
        end
        for k=(length(ch_list)+1):length(ports) %remove ports
            blk_ch = ports{k};
            ind = strfind(blk_ch,'/');
            blk_str = [blk_ch((ind(end)+1):end),'/1'];
            if type_out
                delete_line(blk,blk_str,['Mux/',num2str(k)]);
            else
                delete_line(blk,['Mux/',num2str(k)],blk_str);
            end
            delete_block(blk_ch);
        end
        if type_out
            set_param([blk,'/Mux'],'Inputs',num2str(length(ch_list)));
        else
            set_param([blk,'/Mux'],'Outputs',num2str(length(ch_list)));
        end
        for k=1:(length(ch_list)-length(ports)) % add ports
            blk_ch = [blk,'/','ADD',num2str(k)];
            top = 33+(k+length(ports))*45;
            if type_out
                try add_block(['simulink/Sources/In1'],blk_ch); end
                set_param(blk_ch,'Position',[20 top 50 top+14]);
                add_line(blk,['ADD',num2str(k),'/1'],['Mux/',num2str(k+length(ports))], 'autorouting','on');
            else
                try add_block(['simulink/Sinks/Out1'],blk_ch); end
                set_param(blk_ch,'Position',[420 top 450 top+14]);
                add_line(blk,['Mux/',num2str(k+length(ports))],['ADD',num2str(k),'/1'], 'autorouting','on');
            end
        end
        %rename
        if type_out
            ports = find_system(blk,'LookUnderMasks','all','BlockType','Inport');
        else
            ports = find_system(blk,'LookUnderMasks','all','BlockType','Outport');
        end
        for k=1:length(ch_list)
            set_param(ports{k},'Name',['TMP',num2str(k)])
        end
        if type_out
            ports = find_system(blk,'LookUnderMasks','all','BlockType','Inport');
        else
            ports = find_system(blk,'LookUnderMasks','all','BlockType','Outport');
        end
        for k=1:length(ch_list)
            set_param(ports{k},'Name',[name,num2str(ch_list(k)+offset)])
        end
        blk_all = find_system(bdroot,'LookUnderMasks','all','DrvName',['National_Instruments/',dev]);
        switch type
            case 'AO'
                blk_ad = [blk,'/Analog Output'];
                set_param(blk_ad,'DrvName',['National_Instruments/',dev]);
                set_param(blk_ad,'SampleTime','Ts')
                set_param(blk_ad,'DrvAddress','4294967295') %auto 2^32-1 FFFFFFFF
                DrvOptions = eval(get_param(blk_all{1},'DrvOptions'));
                set_param(blk_ad,'InitialValue','0')
                set_param(blk_ad,'FinalValue','0')
            case 'AI'
                blk_ad = [blk,'/Analog Input'];
                set_param(blk_ad,'DrvName',['National_Instruments/',dev]);
                set_param(blk_ad,'SampleTime','Ts')
                set_param(blk_ad,'DrvAddress','4294967295') %auto 2^32-1 FFFFFFFF
                DrvOptions = eval(get_param(blk_all{1},'DrvOptions'));
                if strcmp(get_param(blk,'connect'),'BNC Terminal')
                    DrvOptions(1) = 1; %diff
                    couple = 'differential';
                else
                    couple = get_param(blk,'couple');
                    if strcmp(couple,'single ended')
                        DrvOptions(1) = 0; %single-ended
                    else
                        DrvOptions(1) = 1; %diff
                    end
                end
            case 'ENC'
                blk_ad = [blk,'/Encoder Input'];
                blk_gain = [blk,'/Gain'];
                set_param(blk_ad,'DrvName',['National_Instruments/',dev]);
                set_param(blk_ad,'SampleTime','Ts')
                set_param(blk_ad,'DrvAddress','4294967295') %auto 2^32-1 FFFFFFFF
                DrvOptions = eval(get_param(blk_all{1},'DrvOptions'));
                DrvOptions(ch_list+3) = 2; % Quad Enc
                set_param(blk_ad,'QuadMode','3')%quadruple
                reset = get_param(blk,'reset');
                if strcmp(reset,'on')
                    set_param(blk_ad,'IndexPulse','5')%reset
                    set_param(gcb,'MaskDisplay','disp(''Encoder\nInput\nIndex Reset On'')');
                else
                    set_param(blk_ad,'IndexPulse','1')%no reset
                    set_param(gcb,'MaskDisplay','disp(''Encoder\nInput\nIndex Reset Off'')');
                end
            case 'CTR'
                blk_ad = [blk,'/Counter Input'];
                blk_fcn = [blk,'/Fcn'];
                set_param(blk_ad,'DrvName',['National_Instruments/',dev]);
                set_param(blk_ad,'SampleTime','Ts')
                set_param(blk_ad,'DrvAddress','4294967295') %auto 2^32-1 FFFFFFFF
                DrvOptions = eval(get_param(blk_all{1},'DrvOptions'));
                DrvOptions(ch_list+3) = 0; % Plain Counter
                mode = get_param(blk,'mode');
                out_mode = get_param(blk,'out_mode');
                resolution = eval(get_param(blk,'resolution'));
                frequ = eval(get_param(blk,'frequ'));
                switch mode
                    case 'Count rising edge'
                        set_param(blk_ad,'CounterEdge','1') %Rising edge
                        set_param(blk_ad,'CounterGate','1') %none
                        switch out_mode
                            case 'Position/Angle [res]'
                                set_param(blk_fcn,'Expr','u(1)*resolution');
                            otherwise
                                set_param(blk_fcn,'Expr','u(1)');
                        end
                    case 'Count falling edge'
                        set_param(blk_ad,'CounterEdge','2') %Falling edge
                        set_param(blk_ad,'CounterGate','1') %none
                        switch out_mode
                            case 'Position/Angle [res]'
                                set_param(blk_fcn,'Expr','u(1)*resolution');
                            otherwise
                                set_param(blk_fcn,'Expr','u(1)');
                        end
                    case 'Measure time between rising edge'
                        set_param(blk_ad,'CounterEdge','3') %100MHz Clock
                        set_param(blk_ad,'CounterGate','10')%Rising edge
                        switch out_mode
                            case 'Frequency [Hz]'
                                set_param(blk_fcn,'Expr','1/(u(1)/100e6)');
                            case 'Velocity/Speed [res/s]'
                                set_param(blk_fcn,'Expr','1/(u(1)/100e6)*resolution');
                            otherwise
                                set_param(blk_fcn,'Expr','u(1)');
                        end
                    case 'Measure time between falling edge'
                        set_param(blk_ad,'CounterEdge','3') %100MHz Clock
                        set_param(blk_ad,'CounterGate','11')%Falling edge
                        switch out_mode
                            case 'Velocity/Speed [res/s]'
                                set_param(blk_fcn,'Expr','1/(u(1)/100e6)*resolution');
                            otherwise
                                set_param(blk_fcn,'Expr','u(1)');
                        end
                    case 'Measure time when high'
                        set_param(blk_ad,'CounterEdge','3') %100MHz Clock
                        set_param(blk_ad,'CounterGate','12')%enable when high
                        switch out_mode
                            case 'Duty Cycle [%]'
                                set_param(blk_fcn,'Expr','(u(1)/100e6*frequ*100');
                            case 'PWM scaled [res]'
                                if resolution<0
                                    set_param(blk_fcn,'Expr','(u(1)/100e6*frequ-0.5)*resolution');
                                else
                                    set_param(blk_fcn,'Expr','u(1)/100e6*frequ*resolution');
                                end
                            otherwise
                                set_param(blk_fcn,'Expr','u(1)/100e6');
                        end
                    case 'Measure time when low'
                        set_param(blk_ad,'CounterEdge','3') %100MHz Clock
                        set_param(blk_ad,'CounterGate','13')%enable when high
                        switch out_mode
                            case 'Duty Cycle [%]'
                                set_param(blk_fcn,'Expr','(u(1)/100e6*frequ*100');
                            case 'PWM scaled [res]'
                                if resolution<0
                                    set_param(blk_fcn,'Expr','(0.5-u(1)/100e6*frequ)*resolution');
                                else
                                    set_param(blk_fcn,'Expr','u(1)/100e6*frequ*resolution');
                                end
                            otherwise
                                set_param(blk_fcn,'Expr','u(1)/100e6');
                        end
                end
                set_param(blk_ad,'ResetMode','1') %never reset
            case 'DI'
                blk_ad = [blk,'/Digital Input'];
                set_param(blk_ad,'DrvName',['National_Instruments/',dev]);
                set_param(blk_ad,'SampleTime','Ts')
                set_param(blk_ad,'DrvAddress','4294967295') %auto 2^32-1 FFFFFFFF
                DrvOptions = eval(get_param(blk_all{1},'DrvOptions'));
                bin = dec2bin(DrvOptions(2),8);
                bin(9-ch_list) = '0';
                DrvOptions(2) = bin2dec(bin); %
            case 'DO'
                blk_ad = [blk,'/Digital Output'];
                set_param(blk_ad,'DrvName',['National_Instruments/',dev]);
                set_param(blk_ad,'SampleTime','Ts')
                set_param(blk_ad,'DrvAddress','4294967295') %auto 2^32-1 FFFFFFFF
                DrvOptions = eval(get_param(blk_all{1},'DrvOptions'));
                bin = dec2bin(DrvOptions(2),8);
                bin(9-ch_list) = '1';
                DrvOptions(2) = bin2dec(bin); %
            case 'PWM'
                blk_ad = [blk,'/Frequency Output'];
                set_param(blk_ad,'DrvName',['National_Instruments/',dev]);
                set_param(blk_ad,'SampleTime','Ts')
                set_param(blk_ad,'DrvAddress','4294967295') %auto 2^32-1 FFFFFFFF
                DrvOptions = eval(get_param(blk_all{1},'DrvOptions'));
                DrvOptions(ch_list+3) = 3; % PWM
                mode = get_param(blk,'mode');
                pwm_frequ = get_param(blk,'pwm_frequ');
                duty = get_param(blk,'duty');
                if strcmp(mode,'PWM')
                    set_param(blk_ad,'DutySource','2') %extern
                    set_param(blk_ad,'FrequencySource','1') %intern
                    set_param(blk_ad,'DutyFinalValue',duty)
                    set_param(blk_ad,'Duty',duty)
                    set_param(blk_ad,'FrequencyFinalValue',pwm_frequ)
                    set_param(blk_ad,'Frequency',pwm_frequ)
                else
                    set_param(blk_ad,'FrequencySource','2') %extern
                    set_param(blk_ad,'DutySource','1') %intern
                    set_param(blk_ad,'DutyFinalValue',duty)
                    set_param(blk_ad,'Duty',duty)
                    set_param(blk_ad,'FrequencyFinalValue',pwm_frequ)
                    set_param(blk_ad,'Frequency',pwm_frequ)
                end
        end
        set_param(gcb,'MaskDisplay',['disp(''NI\n',dev,''')']);
        for k=1:length(blk_all)
            set_param(blk_all{k},'DrvOptions',['[',num2str(DrvOptions),']'])
        end
    case 'bodeoptions'
        hb = get_param(blk,'Handle');
        ud = get(hb,'UserData');
        fig = get_param(blk,'fig');
        
        unwrap_mode = get_param(blk,'unwrap');
        funits = get_param(blk,'funits');
        punits = get_param(blk,'punits');
        aunits = get_param(blk,'aunits');
        fstart = evalin('base',get_param(blk,'fstart'));
        fend = evalin('base',get_param(blk,'fend'));
        
        doit = 0;
        if isempty(ud.h_bode), return; end
        if ~strcmp(ud.unwrap,unwrap_mode), doit=1; end
        if ~strcmp(ud.punits,punits), doit=1; end
        if ~strcmp(ud.aunits,aunits), doit=1; end
        if ud.fstart~=fstart, doit=1; end
        if ud.fend~=fend, doit=1; end
        if ~strcmp(ud.funits,funits),
            doit=1;
            ind = strfind(blk,'/');
            G_str = blk(ind(end)+1:end);
            try
                G = evalin('base',G_str);
                f = G.Frequency;
                Gf = G.ResponseData;
                if strcmp(G.FrequencyUnit,'Hz') && strcmp(funits,'rad/s')
                    G = frd(Gf,f*(2*pi),'FrequencyUnit','rad/s');
                    assignin('base',G_str,G);
                elseif strcmp(G.FrequencyUnit,'rad/s') && strcmp(funits,'Hz')
                    G = frd(Gf,f/(2*pi),'FrequencyUnit','Hz');
                    assignin('base',G_str,G);
                end
            end
        end
        if ~doit
            return
        end
        
        bo = getoptions(ud.h_bode);
        bo.XLimMode = {'auto'};
        bo.YLimMode{1} = 'auto';
        bo.YLimMode{2} = 'auto';
        setoptions(ud.h_bode,bo);
        if strcmp(unwrap_mode,'unwrap phase')
            bo.PhaseWrapping = 'off';
        else
            bo.PhaseWrapping = 'on';
        end
        bo.FreqUnits = funits;
        bo.MagUnits = aunits;
        bo.PhaseUnits = punits;
        if strcmp(funits,'rad/s')
            bo.XLim = {[fstart fend]*2*pi};
        else
            bo.XLim = {[fstart fend]};
        end
        if strcmp(aunits,'dB')
            bo.MagScale = 'linear';
        else
            bo.MagScale = 'log';
        end
        
        setoptions(ud.h_bode,bo);
        figure(eval(fig));
        ZHAWcbfcn('default');
    case 'bode'
        if option || strcmp(get_param(blk,'auto_gpa'),'on')
            hb = get_param(blk,'Handle');
            ud = get(hb,'UserData');
            simstat = get_param(bdroot,'SimulationStatus');
            if strcmp(simstat,'running') || strcmp(simstat,'external')
                set_param(bdroot,'SimulationCommand','WriteDataLogs')
            end
            data = get_param([blk,'/To Workspace'],'VariableName');
            GPA_TD = evalin('base',data);
            fstart = evalin('base',get_param(blk,'fstart'));
            fend = evalin('base',get_param(blk,'fend'));
            funits = get_param(blk,'funits');
            punits = get_param(blk,'punits');
            aunits = get_param(blk,'aunits');
            Wlen_temp = get_param(blk,'Wlen');
            if isempty(Wlen_temp)
                Wlen = [];
            else
                Wlen = eval(Wlen_temp);
            end
            unwrap_mode = strcmp(get_param(blk,'unwrap'),'unwrap phase');
            integ = strcmp(get_param(blk,'integ'),'on');
            hmode = strcmp(get_param(blk,'hmode'),'on');
            fig = get_param(blk,'fig');
            wintyp = get_param(blk,'wintyp');
            if strcmp(get_param(blk,'coherence'),'on')
                calc_cohere = 1;
            else
                calc_cohere = 0;
            end
            if strcmp(funits,'Hz')
                fgain = 1;
            else
                fgain = 2*pi;
            end
            
            trig = GPA_TD.signals.values(:,3);
            ind2 = find(trig==1,1,'last');
            ind1 = find(trig(1:ind2)==0,1,'last');
            if ~ind1
                ind1 = 1;
            end
            xe = GPA_TD.signals.values(ind1:ind2,1);
            xa = GPA_TD.signals.values(ind1:ind2,2);
            t  = GPA_TD.time(ind1:ind2);
            Ts = t(2)-t(1);
            
            if integ
                xa = filter([1 -1],Ts,xa);
            end
            xe = detrend(xe,'constant');
            xa = detrend(xa,'constant');
            if isempty(Wlen)
                NN = length(xe);
                nover = ceil(NN/2);
                [Gf,f] = tfestimate(xe,xa,eval([wintyp,'(NN)']),nover,[],1/Ts);
                if calc_cohere
                    [Gc,f] = mscohere(xe,xa,eval([wintyp,'(NN)']),nover,[],1/Ts);
                end
                NN = floor(NN/5);
                while NN >= 100
                    nover = ceil(NN/2);
                    [Gf2,f2] = tfestimate(xe,xa,eval([wintyp,'(NN)']),nover,[],1/Ts);
                    if calc_cohere
                        [Gc2,f2] = mscohere(xe,xa,eval([wintyp,'(NN)']),nover,[],1/Ts);
                    end
                    fcut = f2(25);
                    i1 = find(f<fcut);
                    i2 = find(f2>=fcut);
                    Gf = [Gf(i1);Gf2(i2)];
                    if calc_cohere
                        Gc = [Gc(i1);Gc2(i2)];
                    end
                    f = [f(i1);f2(i2)];
                    NN = floor(NN/5);
                end
            else
                Wlen = min(Wlen,length(xe));
                nover = ceil(Wlen/2);
                [Gf,f] = tfestimate(xe,xa,eval([wintyp,'(Wlen)']),nover,[],1/Ts);
                if calc_cohere
                    [Gc,f] = mscohere(xe,xa,eval([wintyp,'(Wlen)']),nover,[],1/Ts);
                end
            end
            if integ
                s = j*f*2*pi;
                Gf = Gf./s;
                Gf_a = abs(Gf);
                Gf_p = angle(Gf);
                Gf_p = Gf_p+(2*pi*f)*(Ts/2);
                Gf = Gf_a.*exp(Gf_p*j);
            end
            i1 = find(f<=fstart,1,'last');
            i2 = find(f>=fend,1,'first');
            Nf = length(f);
            if isempty(i1)
                i1 = 1;
            end
            if isempty(i2)
                i2 = length(f);
            end
            Gf = Gf(i1:i2);
            if calc_cohere
                Gc = Gc(i1:i2);
            end
            f  = f(i1:i2);
            f(1) = fstart;
            if i2<Nf
                f(end) = fend;
            end
            G = frd(Gf,f*fgain,'FrequencyUnit',funits);
            
            ind = strfind(blk,'/');
            G_str = blk(ind(end)+1:end);
            assignin('base',G_str,G);
            assignin('base',[G_str,'_td'],GPA_TD);
            if calc_cohere
                assignin('base',[G_str,'_ch'],Gc);
            end
            if unwrap_mode
                PhaseWrapping = 'off';
            else
                PhaseWrapping = 'on';
            end
            if strcmp(aunits,'dB')
                MagUnits = 'dB';
            else
                MagUnits = 'abs';
            end
            if strcmp(punits,'deg')
                PhaseUnits = 'deg';
            else
                PhaseUnits = 'rad';
            end
            try
                hf = eval(fig);
            catch
                hf = findall(0,'Type','figure','Name',fig);
            end
            if ~isempty(hf)
                figure(hf);
                if strcmp(MagUnits,'dB')
                    MagScale = 'linear';
                else
                    MagScale = 'log';
                end
                if ~calc_cohere
                    if strcmp(funits,'rad/s')
                        xlim_ = {[fstart fend]*2*pi};
                    else
                        xlim_ = {[fstart fend]};
                    end
                    if hmode
                        hold on
                    end
                    hl = bodeplot(G);
                    hold off
                    hl.Responses(end).Name = G_str;
                    setoptions(hl,'FreqScale','log',...
                        'FreqUnits',funits,...
                        'MagUnits',MagUnits,...
                        'MagScale',MagScale,...
                        'Grid','on',...
                        'PhaseUnits',PhaseUnits,...
                        'XLim',xlim_,...
                        'XLimMode',{'manual'},...
                        'PhaseWrapping',PhaseWrapping);
                    legend('show');
                    ud.h_bode = hl;
                    set(hb,'UserData',ud);
                else
                    if strcmp(funits,'rad/s')
                        xlim_ = [fstart fend]*2*pi;
                        f_ = f*2*pi;
                    else
                        xlim_ = [fstart fend];
                        f_ = f;
                    end
                    h(1) = subplot(5,1,[1:2]);
                    if hmode
                        hold on
                    end
                    if strcmp(MagUnits,'dB')
                        semilogx(f_,db(squeeze(G.ResponseData)),'DisplayName',G_str)
                    else
                        semilogx(f_,abs(squeeze(G.ResponseData)),'DisplayName',G_str)
                    end
                    hold off
                    grid on
                    ylabel(['Magnitude [',MagUnits,']'])
                    xlim(xlim_)
                    title('Bode Plot')
                    set(gca,'XTickLabel',[])
                    legend('show');
                    h(2) = subplot(5,1,[3:4]);
                    if hmode
                        hold on
                    end
                    if strcmp(PhaseUnits,'deg')
                        semilogx(f_,unwrap(angle(squeeze(G.ResponseData)))*180/pi,'DisplayName',G_str);
                    else
                        semilogx(f_,unwrap(angle(squeeze(G.ResponseData))),'DisplayName',G_str);
                    end
                    hold off
                    xlim(xlim_)
                    grid on
                    ylabel(['Phase [',PhaseUnits,']'])
                    set(gca,'XTickLabel',[])
                    h(3) = subplot(5,1,5);
                    if hmode
                        hold on
                    end
                    semilogx(f_,Gc,'DisplayName',G_str)
                    hold off
                    ylim([-0.05 1.05])
                    ylabel('Coherence')
                    xlabel(['Frequency [',funits,'])'])
                    xlim(xlim_)
                    grid on
                    linkaxes(h,'x')
                end
            end
        end
    case 'copySPEC'
        set_param(blk,'LinkStatus',getpref('ZHAW_SLDRT_lib','LibraryLinkStatus','none'));
        %set_param(blk,'OpenFcn','open_system(gcb,''Mask'')')
        set_param([blk,'/To Workspace'],'SaveFormat','StructureWithTime');
        funits = getpref('ZHAW_SLDRT_lib','funits','Hz');
        set_param(blk,'funits',funits);
        aunits = getpref('ZHAW_SLDRT_lib','aunits','dB');
        set_param(blk,'aunits',aunits);
        punits = getpref('ZHAW_SLDRT_lib','punits','deg');
        set_param(blk,'punits',punits);
        try
            set_param(gcb,'Name','X')
        catch
            notok = 1;
            while notok
                try
                    set_param(gcb,'Name',['X',num2str(notok)])
                    notok = 0;
                catch
                    notok = notok+1;
                end
            end
            clear notok
        end
    case 'copyGPA'
        set_param(blk,'LinkStatus',getpref('ZHAW_SLDRT_lib','LibraryLinkStatus','none'));
        set_param([blk,'/To Workspace'],'SaveFormat','StructureWithTime');
        funits = getpref('ZHAW_SLDRT_lib','funits','Hz');
        set_param(blk,'funits',funits);
        aunits = getpref('ZHAW_SLDRT_lib','aunits','dB');
        set_param(blk,'aunits',aunits);
        punits = getpref('ZHAW_SLDRT_lib','punits','deg');
        set_param(blk,'punits',punits);
        unwrap_mode = getpref('ZHAW_SLDRT_lib','unwrap','wrap phase +/-180°');
        set_param(blk,'unwrap',unwrap_mode);
        fs_max_str = getpref('ZHAW_SLDRT_lib','max_sampling_frequency','20000');
        try
            evalin('base','Ts;');
        catch
            FixedStep = get_param(bdroot,'FixedStep');
            if strcmp(FixedStep,'auto')
                FixedStep = ['1/',fs_max_str];
            end
            assignin('base','Ts',eval(FixedStep));
        end
        try
            set_param(gcb,'Name','G')
        catch
            notok = 1;
            while notok
                try
                    set_param(gcb,'Name',['G',num2str(notok)])
                    notok = 0;
                catch
                    notok = notok+1;
                end
            end
            clear notok
        end
        ZHAWcbfcn('update');
        ZHAWcbfcn('default');
    case 'ChirpMode'
        blk_chirp = find_system(bdroot,'MaskType','bode_chirp');
        if ~isempty(blk_chirp)
            blk = blk_chirp{1};
            mode = get_param(blk,'mode');
            switch mode
                case 'linear'
                    mode_n = 1;
                case 'logarithmic'
                    mode_n = 2;
                case 'quadratic'
                    mode_n = 3;
            end
            set_param(blk,'mode_n',num2str(mode_n));
            ZHAWcbfcn('ApplyChanges',blk);
        end
    case 'SettleTime'
        blk_gpa = find_system(bdroot,'MaskType','gpa');
        blk_chirp = find_system(bdroot,'MaskType','bode_chirp');
        for k=1:length(blk_gpa)
            if strcmp(get_param(blk_gpa{k},'editTsettle'),'off') && ~isempty(blk_chirp)
                Tsettle = evalin('base',get_param(blk_chirp{1},'T0'));
                if ~strcmp(get_param(blk_gpa{k},'Tsettle'),num2str(Tsettle))
                    set_param(blk_gpa{k},'Tsettle',num2str(Tsettle));
                end
                ZHAWcbfcn('ApplyChanges',blk_gpa{k});
            end
        end
    case {'edit_fstart','edit_fend'}
        MaskEnables = get_param(blk,'MaskEnables');
        if strcmp(get_param(blk,'edit_fstart'),'off')
            MaskEnables{4} = 'off';
        else
            MaskEnables{4} = 'on';
        end
        if strcmp(get_param(blk,'edit_fend'),'off')
            MaskEnables{6} = 'off';
        else
            MaskEnables{6} = 'on';
        end
        set_param(blk,'MaskEnables',MaskEnables);
    case 'fstart'
        blk_gpa = find_system(bdroot,'MaskType','gpa');
        blk_chirp = find_system(bdroot,'MaskType','bode_chirp');
        if ~isempty(blk_chirp)
            blk = blk_chirp{1};
            f1 = evalin('base',get_param(blk,'f1'));
            f2 = evalin('base',get_param(blk,'f2'));
            if f2<f1
                warndlg('end frequency must be larger than start frequency.')
                set_param(blk,'f1',num2str(f2));
                set_param(blk,'f2',num2str(f1));
            end
            for k=1:length(blk_gpa)
                if strcmp(get_param(blk_gpa{k},'edit_fstart'),'off')
                    if ~strcmp(get_param(blk_gpa{k},'fstart'),num2str(f1))
                        set_param(blk_gpa{k},'fstart',num2str(f1));
                    end
                end
                if strcmp(get_param(blk_gpa{k},'edit_fend'),'off')
                    if ~strcmp(get_param(blk_gpa{k},'fend'),num2str(f2))
                        set_param(blk_gpa{k},'fend',num2str(f2));
                    end
                end
                ZHAWcbfcn('ApplyChanges',blk_gpa{k});
            end
        end
    case 'fend'
        blk_gpa = find_system(bdroot,'MaskType','gpa');
        blk_chirp = find_system(bdroot,'MaskType','bode_chirp');
        if ~isempty(blk_chirp)
            blk = blk_chirp{1};
            f1 = evalin('base',get_param(blk,'f1'));
            f2 = evalin('base',get_param(blk,'f2'));
            Ts = evalin('base',get_param(bdroot,'FixedStep'));
            fn = 1/Ts/2;
            if (f2/fn-1)>eps
                set_param(blk,'f2',num2str(fn));
                f2 = fn;
                warndlg('End frequency too large (fend<=fs/2).')
            end
            if f2<f1
                warndlg('end frequency must be larger than start frequency.')
                set_param(blk,'f1',num2str(f2));
                set_param(blk,'f2',num2str(f1));
            end
            for k=1:length(blk_gpa)
                if strcmp(get_param(blk_gpa{k},'edit_fstart'),'off')
                    if ~strcmp(get_param(blk_gpa{k},'fstart'),num2str(f1))
                        set_param(blk_gpa{k},'fstart',num2str(f1));
                    end
                end
                if strcmp(get_param(blk_gpa{k},'edit_fend'),'off')
                    if ~strcmp(get_param(blk_gpa{k},'fend'),num2str(f2))
                        set_param(blk_gpa{k},'fend',num2str(f2));
                    end
                end
                ZHAWcbfcn('ApplyChanges',blk_gpa{k});
            end
            ZHAWcbfcn('ApplyChanges',blk);
        end
    case {'gpafstart','gpafend'}
        f1 = evalin('base',get_param(blk,'fstart'));
        f2 = evalin('base',get_param(blk,'fend'));
        if f2<f1
            warndlg('end frequency must be larger than start frequency.')
            set_param(blk,'fstart',num2str(f2));
            set_param(blk,'fend',num2str(f1));
        end
    case 'editTsettle'
        MaskEnables = get_param(blk,'MaskEnables');
        if strcmp(get_param(blk,'editTsettle'),'off')
            MaskEnables{11} = 'off';
        else
            MaskEnables{11} = 'on';
        end
        set_param(blk,'MaskEnables',MaskEnables);
    case 'editSweepTime'
        blk_chirp = find_system(bdroot,'MaskType','bode_chirp');
        if ~isempty(blk_chirp)
            blk = blk_chirp{1};
            MaskEnables = get_param(blk,'MaskEnables');
            if strcmp(get_param(blk,'editSweepTime'),'off')
                f1 = evalin('base',get_param(blk,'f1'));
                if ~strcmp(get_param(blk,'Tm'),num2str(1/f1))
                    set_param([blk,'/Tm'],'Value',num2str(1/f1))
                    set_param(blk,'Tm',num2str(1/f1));
                end
                MaskEnables{7} = 'off';
            else
                MaskEnables{7} = 'on';
            end
            %if ~strcmp(get_param(bdroot,'SystemTargetFile'),'bur_grt.tlc')
                set_param(blk,'MaskEnables',MaskEnables); %****
            %end
            ZHAWcbfcn('ApplyChanges',blk);
        end
    case 'SimTime'
        blk_chirp = find_system(bdroot,'MaskType','bode_chirp');
        blk_bode = find_system(bdroot,'MaskType','gpa');
        if ~isempty(blk_bode)
            blk_bode = blk_bode{1};
        end
        if ~isempty(blk_chirp)
            blk = blk_chirp{1};
            Tm = evalin('base',get_param(blk,'Tm'));
            T0 = evalin('base',get_param(blk,'T0'));
            Te = T0 + Tm;
            set_param([blk,'/Te'],'Value',num2str(Te))
            set_param([blk,'/Tm'],'Value',num2str(Tm))
            simstat = get_param(bdroot,'SimulationStatus');
            if  strcmp(get_param(bdroot,'SystemTargetFile'),'bur_grt.tlc')
                set_param([blk,'/Trigger'],'Value','0')
            else
                set_param([blk,'/Trigger'],'Value','1') %****
                if strcmp(simstat,'initializing') || strcmp(simstat,'stopped') || strcmp(simstat,'terminating')
                    set_param(bdroot,'StopTime',num2str(Te)); %****
                end
            end
            ZHAWcbfcn('ApplyChanges',blk);
            ZHAWcbfcn('ApplyChanges',blk_bode);
        end
    case 'ApplyChanges' 
        if option
            blk = option;
        end
        blk_handle = get_param(blk,'Handle');
        dlgs = DAStudio.ToolRoot.getOpenDialogs;
        for i=1:length(dlgs) %find dialog of selected block
            if class(dlgs(i).getSource) == "Simulink.SLDialogSource"
                dialog_block_handle = dlgs(i).getSource.getBlock.Handle;
                if dialog_block_handle == blk_handle
                    dlgs(i).apply %'click' apply
                end
            end
        end
    case 'Spec'
        simstat = get_param(bdroot,'SimulationStatus');
        if strcmp(simstat,'stopped') || strcmp(simstat,'terminating')
            data = get_param([blk,'/To Workspace'],'VariableName');
            try
                var = evalin('base',data);
                if isstruct(var)
                    t = var.time;
                    x = var.signals.values;
                else
                    t = var.Time;
                    x = var.Data;
                end
                if length(t)<2
                    return
                end
            catch
                return
            end
            x = x(1:end-1,:);
            t = t(1:end-1);
            plot_phase = strcmp(get_param(blk,'phase'),'on');
            plot_amp = strcmp(get_param(blk,'amp'),'on');
            flog = strcmp(get_param(blk,'flog'),'on');
            funits = get_param(blk,'funits');
            punits = get_param(blk,'punits');
            aunits = get_param(blk,'aunits');
            wintyp = get_param(blk,'wintyp');
            fig = get_param(blk,'fig');
            x = eval(['x.*',wintyp,'(length(x));']);
            [Fk_,f_,Ck_,Pk_] = my_fft(t,x);
            ind = strfind(blk,'/');
            sys = blk(ind(end)+1:end);
            if strcmp(funits,'rad/s')
                f_ = f_*2*pi;
                flab = 'Frequency [rad/s]';
            else
                flab = 'Frequency [Hz]';
            end
            if strcmp(aunits,'dB')
                alab = 'Magnitude [dB]';
                Ck_ = db(Ck_);
            else
                alab = 'Magnitude';
            end
            if strcmp(punits,'deg')
                Pk_ = Pk_*180/pi;
                plab = 'Phase [deg]';
            else
                plab = 'Phase [rad]';
            end
            %plot spectrum
            if plot_amp || plot_phase
                hf = [];
                try
                    hf = eval(fig);
                catch
                    hf = findall(0,'Type','figure','Name',fig);
                end
                if ~isempty(hf)
                    figure(hf);
                    %set(hf,'Tag','Spectrum','Name','Spectrum','NumberTitle','off');
                    if plot_amp
                        ha = findall(hf,'Type','axes','Tag','AmplitudeAxes');
                        h_mode = strcmp(get_param(blk,'hold_mode'),'on') && ~isempty(ha);
                        if ~h_mode
                            if plot_phase
                                ha = subplot(2,1,1,'replace');
                            else
                                ha = subplot(1,1,1,'replace');
                            end
                        else
                            ha = ha(1);
                            axes(ha);
                        end
                        Ck_(1) = 0;
                        if h_mode
                            hold on
                        end
                        if flog
                            for k=1:size(Ck_,2)
                                semilogx(f_,Ck_(:,k),'Color',getnextcolor,'DisplayName',sys);
                                hold on
                            end
                        else
                            for k=1:size(Ck_,2)
                                plot(f_,Ck_(:,k),'Color',getnextcolor,'DisplayName',sys);
                                hold on
                            end
                        end
                        set(ha,'Tag','AmplitudeAxes');
                        hold off
                        ylabel(alab)
                        grid on
                    end
                    if plot_phase
                        ha = findall(hf,'Type','axes','Tag','PhaseAxes');
                        h_mode = strcmp(get_param(blk,'hold_mode'),'on') && ~isempty(ha);
                        if ~h_mode
                            if plot_amp
                                ha = subplot(2,1,2,'replace');
                            else
                                ha= subplot(1,1,1,'replace');
                            end
                        else
                            ha = ha(1);
                            axes(ha);
                        end
                        if h_mode
                            hold on
                        end
                        if flog
                            for k=1:size(Pk_,2)
                                semilogx(f_,Pk_(:,k),'Color',getnextcolor,'DisplayName',sys);
                                hold on
                            end
                        else
                            for k=1:size(Pk_,2)
                                plot(f_,Pk_(:,k),'Color',getnextcolor,'DisplayName',sys);
                                hold on
                            end
                        end
                        set(ha,'Tag','PhaseAxes');
                        hold off
                        ylabel(plab)
                        grid on
                    end
                    xlabel(flab)
                    legend('show');
                end
            end
            G.Frequency = f_;
            G.ResponseData = Fk_;
            G.Amplitude = Ck_;
            G.Phase = Pk_;
            G.FrequUnits = funits;
            G.PhaseUnits = punits;
            G.AmplitudeUnits = aunits;
            G.Unwrap = 'off';
            assignin('base',sys,G);
        end
    case 'defaultchirp'
        %set_param(blk,'OpenFcn','open_system(gcb,''Mask'')')
        hb = get_param(blk,'Handle');
        ud = get(hb,'UserData');
        f1 = evalin('base',get_param(blk,'f1'));
        f2 = evalin('base',get_param(blk,'f2'));
        Tsettle = evalin('base',get_param(blk,'Tsettle'));
        ud.f1 = f1;
        ud.f2 = f2;
        ud.Tsettle = Tsettle;
        ud.Te = 1/f1+Tsettle;
        set(hb,'UserData',ud);
    case 'default'
        hb = get_param(blk,'Handle');
        ud = get(hb,'UserData');
        unwrap_mode = get_param(blk,'unwrap');
        funits = get_param(blk,'funits');
        punits = get_param(blk,'punits');
        aunits = get_param(blk,'aunits');
        fstart = evalin('base',get_param(blk,'fstart'));
        fend = evalin('base',get_param(blk,'fend'));
        %shiftphase = evalin('base',get_param(blk,'shiftphase'));
        Tsettle = evalin('base',get_param(blk,'Tsettle'));
        if ~isfield(ud,'h_bode')
            ud.h_bode = [];
        end
        ud.unwrap = unwrap_mode;
        ud.funits = funits;
        ud.punits = punits;
        ud.aunits = aunits;
        ud.fstart = fstart;
        ud.fend = fend;
        %ud.shiftphase = shiftphase;
        ud.Tsettle = Tsettle;
        set(hb,'UserData',ud);
end
end
function [Fk, f, Ck, Pk] = my_fft(t,x)
Ts = t(2)-t(1);
N = length(x);
T0 = N*Ts;
df = 1/T0;
n = floor((N+1)/2);
Y = fft(x);
Fk = Y(1:n,:)/N;
f  = (0:n-1)'*df;
Ak = 2*real(Fk);
Ak(1) = Fk(1);
Bk = -2*imag(Fk);
Ck = 2*abs(Fk);
Ck(1) = Fk(1);
Pk = -atan2(Bk,Ak);
tol = 1e-10;
Pk(Ck<tol) = 0;
end
function col = getnextcolor
ha = gca;
c_order = get(ha,'ColorOrder');
if strcmp(get(ha,'NextPlot'),'replace')
    col = c_order(1,:);
else
    hl = get(ha,'Children');
    ind = mod(length(hl),size(c_order,1));
    col = c_order(ind+1,:);
end
end
function [type,name,type_out,dev,N_CH,ch_list,offset] = getdeviceinfo(mode)
dev = get_param(gcb,'device');
MaskType = get_param(gcb,'MaskType');
switch MaskType
    case 'NI Analog Output'
        type = 'AO';
        type_out = 1;
        switch dev
            case {'PCI-6221','PCIe-6321'}
                N_CH = 2;
            case {'PCI-6259','PCIe-6259','PCIe-6363'}
                N_CH = 4;
        end
    case 'NI PWM Output'
        type = 'PWM';
        type_out = 1;
        switch dev
            case {'PCI-6221','PCI-6259','PCIe-6259'}
                N_CH = 2;
            case {'PCIe-6321','PCIe-6363'}
                N_CH = 4;
        end
    case 'NI Frequ Output'
        type = 'Frequ';
        type_out = 1;
        switch dev
            case {'PCI-6221','PCI-6259','PCIe-6259'}
                N_CH = 2;
            case {'PCIe-6321','PCIe-6363'}
                N_CH = 4;
        end
    case 'NI Analog Input'
        type = 'AI';
        type_out = 0;
        if strcmp(get_param(gcb,'connect'),'BNC Terminal')
            couple = 'differential';
        else
            couple = get_param(gcb,'couple');
        end
        if strcmp(couple,'single ended')
            N_CH = 16;
            differential = 0;
        else
            N_CH = 8;
            differential = 1;
        end
    case 'NI Encoder Input'
        type = 'ENC';
        type_out = 0;
        switch dev
            case {'PCI-6221','PCI-6259','PCIe-6259'}
                N_CH = 2;
            case {'PCIe-6321','PCIe-6363'}
                N_CH = 4;
        end
    case 'NI Counter Input'
        type = 'CTR';
        type_out = 0;
        switch dev
            case {'PCI-6221','PCI-6259','PCIe-6259'}
                N_CH = 2;
            case {'PCIe-6321','PCIe-6363'}
                N_CH = 4;
        end
    case 'NI Digital Input'
        type = 'DI';
        type_out = 0;
        N_CH = 8;
    case 'NI Digital Output'
        type = 'DO';
        type_out = 1;
        N_CH = 8;
end
switch get_param(gcb,'connect')
    case {'BNC Terminal','Screw Terminal'}
        name = type;
        offset = -1;
    case 'Banana Terminal'
        switch MaskType
            case {'NI Digital Input'}
                name = 'DI';
            case {'NI Digital Output'}
                name = 'DO';
            case {'NI Counter Input','NI Encoder Input','NI PWM Output'}
                name = 'ENC';
            otherwise
                name = 'CH';
        end
        offset = 0;
end
ch_list = [];
for k=1:N_CH
    if strcmp(get_param(gcb,['ch',num2str(k)]),'on')
        ch_list = [ch_list,k];
    end
end
if nargin==1
    switch mode
        case 'MaskType'
            type = MaskType;
        case 'name'
            type = name;
        case 'dev'
            type = dev;
        case 'N_CH'
            type = N_CH;
        case 'ch_list'
            type = ch_list;
        case 'offset'
            type = offset;
        case 'differential'
            type = differential;
        otherwise
    end
end
end
function [y,p]=num2sci(x,n)
%NUM2SCI Convert number to scientific format.
% y -24 Y 24
% z -21 Z 21
% a -18 E 18
% f -15 P 15
% p -12 T 12
% n -9 G 9
% u -6 M 6
% m -3 k 3
step=3;
pfix='yzafpnum kMGTPEZY';
f=(1+5*step*eps);
if ~isnan(x) & x
    e=floor(log10(abs(x*f)));
else
    e=0;
end
t=e-mod(e,step);
if -8*step<=t & t<9*step
    if nargin<3
        y=num2str(x/10^t);
    else
        y=num2str(x/10^t,n);
    end
    p=pfix((9*step+t)/step);
    if p==' ',p='';end
    if nargout<2
        y=[y ' ' p];
    end
else
    if nargin<2
        y=num2str(x);
    else
        y=num2str(x,n);
    end
    p='';
end
end
function Install_NI_Device(dev)
if nargin==0
    dev = getpref('ZHAW_SLDRT_lib','device','PCIe-6321');
end
% build the board structure
brd.DrvName = ['National_Instruments/',dev];
brd.DrvAddress = (2^32-1); %4294967295 (auto)
% add the board to the boards array
brdlist = getpref('RealTimeWindowsTarget', 'InstalledBoards', struct('DrvName',{},'DrvAddress',{}));
for k=1:length(brdlist)
    if strcmp(brdlist(k).DrvName,brd.DrvName)
        return
    end
end
boards = [brdlist brd];
if ~isempty(boards)
    [~, srt] = sort([boards.DrvAddress]);
    boards = boards(srt);
    [~, srt] = sort({boards.DrvName});
    boards = boards(srt);
end
setpref('RealTimeWindowsTarget','InstalledBoards', boards);
end