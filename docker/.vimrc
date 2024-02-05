"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" General
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

" :set spell
" :set spelllang=nl

set t_8f=[38;2;%lu;%lu;%lum
set t_8b=[48;2;%lu;%lu;%lum

"Get out of VI's compatible mode..
"gets rid of all the crap that Vim does to be vi compatible. 
set nocompatible

"Sets how many lines of history VIM has to remember
set history=9999

" -----------------------------------------------------------------------------
" Each time a new or existing file is edited, Vim will try to recognize the type
" of the  file and  set the  'filetype' option. This  will trigger  the FileType
" event, which can be used to set the syntax highlighting, set options, etc.
" plugin
"
"     This actually loads the file "ftplugin.vim" in 'runtimepath'.
"
"         The result is that when a file is edited its plugin file is loaded (if
"         there is one for the detected filetype).
"
" indent 
"
"
"    This actually loads the file "indent.vim" in 'runtimepath'.
"
"    The result  is that when  a file  is edited its  indent file is  loaded (if
" there is one for the detected filetype). indent-expression
"
filetype plugin indent on


" -----------------------------------------------------------------------------
" Set to auto read when a file is changed from the outside
" Autoread  does not  reload  file unless  you do  something  like run  external
" command (like  !ls or  !sh etc) vim  does not do  checks periodically  you can
" reload file manually using :e
set autoread


" -----------------------------------------------------------------------------
" Have the mouse enabled all the time:
" Normally, if  you try to copy  text out of the  xterm that vim is  running in,
" you'll get the text  as well as the numbers. The GUI  version gets this right:
" it only selects the  text, keeping the line numbers out of  the picture. But I
" don't
" want the GUI version. So instead, I added this to my vimrc:
"
" :set mouse=a
"
" Much better. You can also selectively  enable mouse support for specific modes
" only by using something other than 'a' (for 'all').
set mouse=a


" -----------------------------------------------------------------------------
" Prevents some security exploits having to do with modelines in files. 
" https://www.techrepublic.com/blog/it-security/turn-off-modeline-support-in-vim/
set modelines=0
set nomodeline

" Tabs are 4 spaces wide and are spaces
set tabstop=4
set shiftwidth=4
set softtabstop=4
set expandtab

set encoding=utf-8

set visualbell
set cursorline
set ttyfast
set ruler

set backspace=indent,eol,start
set laststatus=2

if $TERM == "xterm-256color"
    set t_Co=256
endif

"show matching bracets
set showmatch

"How many tenths of a second to blink
set mat=1


"Make the new window appears below the current window.
:se splitbelow 

"Make the new window appears in right. (only 6.0 version can do a vsplit)
:se splitright

"Improve the statusline.
" Normally not visible, powerline wil be used.
" See Vundle Plugins
" set statusline=%F%m%r%h%w\ [FORMAT=%{&ff}]\ [TYPE=%Y]\ [ASCII=\%03.3b]\ [HEX=\%02.2B]\ [POS=%04l,%04v][%p%%]\ [LEN=%L]

"Make place for the statusline, so it's always there.
set laststatus=2 

"Make the menubar and toolbar toggeble.
map <silent> <C-F2> :if &guioptions =~# 'T' <Bar>
                \set guioptions-=T <Bar>
                \set guioptions-=m <Bar>
            \else <Bar>
                \set guioptions+=T <Bar>
                \set guioptions+=m <Bar>
            \endif<CR> 


" set guioptions-=r

" Make the scrollbars toggable
" map <silent> <C-F3> :if &guioptions =~# 'r' <Bar>
"                 \set guioptions-=r <Bar>
"            \else <Bar>
" \set guioptions+=r <Bar>
"            \endif<CR>

" Turn off useless toolbar
" set guioptions-=T

" Turn off menu bar (toggle with CTRL+F11)
" set guioptions-=m

" Turn off right-hand scroll-bar (toggle with CTRL+F7)
" set guioptions-=r

" Turn off left-hand scroll-bar (toggle with CTRL+F7)
" set guioptions-=l
" set guioptions-=L

" CTRL+F11 to toggle the menu bar
nmap <C-F11> :if &guioptions=~'m' \| set guioptions-=m \| else \| set guioptions+=m \| endif<CR>

" CTRL+F7 to toggle the right-hand scroll bar
nmap <C-F7> :if &guioptions=~'r' \| set guioptions-=r \| else \| set guioptions+=r \| endif<CR>

" CTRL+F6 to toggle the left-hand scroll bar
nmap <C-F6> :if &guioptions=~'lL' \| set guioptions-=lL \| else \| set guioptions+=lL \| endif<CR>

" CTRL+F5 to toggle the toolbar
nmap <C-F5> :if &guioptions=~'T' \| set guioptions-=T \| else \| set guioptions+=T \| endif<CR>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Files and backups
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"Turn backup off
set nobackup
set nowb
set noswapfile


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Text options
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
   """"""""""""""""""""""""""""""
   " Indent
   """"""""""""""""""""""""""""""
   "Auto indent
   set ai

   "Smart indent
   set si

   "C-style indeting
   set cindent

   "Wrap lines
   set wrap


set tabstop=4
set expandtab
set shiftwidth=4

"set textwidth=80
set nu             " Turn the linenumbers on
"set noai           " Turn the automatic indentation off
set showmatch      " show the matching braces, . . .
set wildmode=longest,list " Make tab-completion work more like bash
set noerrorbells
set vb t_vb=
set ruler
set smarttab autoindent
set smartindent

set backspace=indent,eol,start " Provide a natural backspace
set nocp
filetype plugin on
"let g:snip_set_textmate_cp = 1 " Textmate compatibility 

" Keyboard Shortcuts
map <F6> :Tlist<CR>


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Text options
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"Iabbr fori for <datum> in <data>:<CR><datum> <:system('date')><CR><CR>Cursor here:<> 

vmap <C-C> "+y
nmap <C-V> "+gP
imap <C-V> <ESC

filetype off                  " required

" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim
call vundle#begin()

" let Vundle manage Vundle, required
Plugin 'VundleVim/Vundle.vim'

" Docker
Plugin 'ekalinin/Dockerfile.vim'

Bundle 'sonph/onehalf', {'rtp': 'vim/'}

Plugin 'huyvohcmc/atlas.vim'

" added nerdtree
Plugin 'scrooloose/nerdtree'

" Vim Airline
" lean & mean status/tabline for vim that's light as air 
Plugin 'vim-airline/vim-airline'
Plugin 'vim-airline/vim-airline-themes'

Plugin 'bling/vim-bufferline'

Plugin 'vim-syntastic/syntastic'

Plugin 'vim-scripts/taglist.vim'

Plugin 'wincent/command-t'

" Gruvbox theme
Plugin 'morhetz/gruvbox'

" Distraction-free writing in Vim.
Plugin 'junegunn/goyo.vim'

" I'm not going to lie to you; fugitive.vim may very well be the best Git 
" wrapper of all time. 
Plugin 'tpope/vim-fugitive'

" Syntax highlighting, matching rules and mappings for the original Markdown 
" and extensions.
Plugin 'plasticboy/vim-markdown'

" This plugin is used for displaying thin vertical lines at each indentation 
" level for code indented with spaces.
Plugin 'yggdroot/indentline'

" Hyperfocus-writing in Vim.
" https://github.com/junegunn/limelight.vim
Plugin 'junegunn/limelight.vim'

Plugin 'sheerun/vim-polyglot'


Plugin 'dhruvasagar/vim-table-mode'


" When you  wish to  unload a  file from  the buffer  and keep  the window/split
" intact:
" :BUN

" When you  wish to  delete a  file from  the buffer  and keep  the window/split
" intact:
" :BD

" When you wish to wipe a file from the buffer and keep the window/split intact:
" :BW

" Notice how the key mappings are the  uppercase version of the :bun :bd :bw Vim
" commands? Easy!
Plugin 'qpkorr/vim-bufkill'

Plugin 'twe4ked/vim-colorscheme-switcher'

" All of your Plugins must be added before the following line
call vundle#end()            " required
filetype plugin indent on    " required

let g:goyo_width = '60%'

" air-line
let g:airline_powerline_fonts = 1

if !exists('g:airline_symbols')
    let g:airline_symbols = {}
endif

" unicode symbols
let g:airline_left_sep = '»'
let g:airline_left_sep = '▶'
let g:airline_right_sep = '«'
let g:airline_right_sep = '◀'
let g:airline_symbols.linenr = '␊'
let g:airline_symbols.linenr = '␤'
let g:airline_symbols.linenr = '¶'
let g:airline_symbols.branch = '⎇'
"let g:airline_symbols.paste = 'ρ'
"let g:airline_symbols.paste = 'Þ'
"let g:airline_symbols.paste = '∥'
let g:airline_symbols.whitespace = 'Ξ'

" airline symbols
let g:airline_left_sep = ''
let g:airline_left_alt_sep = ''
let g:airline_right_sep = ''
let g:airline_right_alt_sep = ''
let g:airline_symbols.branch = ''
let g:airline_symbols.readonly = ''
let g:airline_symbols.linenr = ''

" To ignore plugin indent changes, instead use:
"filetype plugin on
"
" Brief help
" :PluginList       - lists configured plugins
" :PluginInstall    - installs plugins; append `!` to update or just :PluginUpdate
" :PluginSearch foo - searches for foo; append `!` to refresh local cache
" :PluginClean      - confirms removal of unused plugins; append `!` to auto-approve removal
"
" see :h vundle for more details or wiki for FAQ
" Put your non-Plugin stuff after this line

" Make Cut Copy Paste work in Unity with CtrlX, CtrlC and CtrlV
" http://superuser.com/questions/10588/how-to-make-cut-copy-paste-in-gvim-on-ubuntu-work-with-ctrlx-ctrlc-ctrlv
source $VIMRUNTIME/mswin.vim
behave mswin



"set statusline+=%#warningmsg#
"set statusline+=%{SyntasticStatuslineFlag()}
"set statusline+=%*

let g:syntastic_always_populate_loc_list = 1
let g:syntastic_auto_loc_list = 1
let g:syntastic_check_on_open = 1
let g:syntastic_check_on_wq = 0

let mapleader = ","

" nnoremap / /\v
" vnoremap / /\v
set ignorecase
set smartcase
" gdefault  applies substitutions  globally on  lines. For  example, instead  of
" :%s/foo/bar/g you just type :%s/foo/bar/
set visualbell
set cursorline
set ttyfast
set ruler

set backspace=indent,eol,start
set laststatus=2

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" VIM userinterface
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
set gdefault
set incsearch
set showmatch
"Highlight search things
set hlsearch
nnoremap <leader><space> :noh<cr>
nnoremap <tab> %
vnoremap <tab> %

set wrap
set textwidth=120
set formatoptions=qrn1


" Normally <F1> gives you the Help.
" When it gets hit on, you prob. wannted the <ESC>.
" Let's remap <F1> to <ESC> ;-)
inoremap <F1> <ESC>
nnoremap <F1> <ESC>
vnoremap <F1> <ESC>

" Hah you forgot to press the Shift, no worries, I've got your back!
nnoremap ; :

set omnifunc=syntaxcomplete#Complete
set formatprg=par\ -w80rj 

set fillchars+=vert:│
"autocmd ColorScheme * highlight VertSplit cterm=NONE ctermfg=NONE ctermbg=NONE guibg=NONE
hi VertSplit ctermbg=NONE guibg=NONE
set shortmess+=c

set foldlevelstart=99

"vmap <C-C> "+y
""nmap <C-V> "+gP
"imap <C-V> <ESC
command! Visual      normal! v
command! VisualLine  normal! V
command! VisualBlock execute "normal! \<C-v>"

"Enable colors and syntax highlighting
set term=screen-256color

set colorcolumn=120
highlight ColorColumn ctermbg=222 guibg=#222222
execute "set colorcolumn=" . join(range(85,335), ',')
let &colorcolumn="80,".join(range(121,999),",")

function! THEME(kind)
    if a:kind == "Light"
        syntax on
        set background=light
        set bg=light
        set t_Co=256
        set cursorline
        colorscheme onehalflight
    elseif a:kind == "dark"
        syntax on
        syntax enable
        set background=dark
        set bg=dark
        set t_Co=256
        set cursorline
        colorscheme gruvbox
    else
        echom "Non-existing theme..."
    endif
endfunction

command! THEMELIGHT :call THEME("light")
command! THEMEDARK  :call THEME("dark")

if $VIM_DARK == 1
    THEMELIGHT
else
    THEMEDARK
endif
