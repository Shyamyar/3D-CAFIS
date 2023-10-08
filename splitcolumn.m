function varargout = splitcolumn(varargin)
    var = num2cell(varargin{1},1);
    if nargin==1
        cols = 1:nargout;
    elseif nargin==2
        cols = varargin{2};
    end
        varargout = var(cols);
end