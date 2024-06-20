function a = extra(width,varargin)
    p = inputParser;
    
    p.addRequired('width', @isnumeric); % necessary argument
    p.addOptional('height',@isnumeric); % Optional argument
    p.addOptional('breadth', @isnumeric);
    p.parse(width,varargin{:});
    
    try 
        a = p.Results.width * p.Results.height; 
    catch
        a = p.Results.width;
    end
end